
// ---------------------------------------------------------------------------------------------------------------------
// x1s heat guard (owner's rule: the phone powers off at 70 C). Ported from the iOS 16 S20 kernel's
// drivers/power/hakios.exynos_thermal.c, which ran on this phone (11 Nov 2026: zones read, power-off tested).
// The Exynos 990's temperature sensors belong to the APM (Samsung ACPM firmware); it is asked through its message queues
// in SRAM 0x02039000 and the mailbox 0x15900000, channel 9 (TMU), {0, 0x02 | zone << 16}: zone 0 BIG, 1 MID, 2 LITTLE,
// 3 G3D, the answer's top byte in whole degrees. Every second; on two readings in a row at or above 70 C (CPU) or 60 C
// (GPU), or no reading for 10 s, the phone is powered off the way Samsung's kernel does: the power chip's restart features
// WTSR/SMPL off (APM channel 2), the APM told to stop (channel 4), PS_HOLD low (PMU 0x1586030C bit 8).
// Boot-args: x1s_heat_stop=<C> (20..70, lower only), x1s_heat_gpu_stop=<C> (20..70).
// ---------------------------------------------------------------------------------------------------------------------
#define X1S_ACPM_SRAM_PA   0x02039000ULL
#define X1S_ACPM_SRAM_SIZE 0x38000U
#define X1S_ACPM_INITDATA  0x7000U
#define X1S_ACPM_MBOX_PA   0x15900000ULL
#define X1S_PMU_PA         0x15860000ULL
#define X1S_SEQ_SHIFT      16
#define X1S_SEQ_MASK       (0x3fU << X1S_SEQ_SHIFT)

struct X1sAcpmChannel {
  uint32_t id, field;
  int32_t owner;
  uint32_t type;
  uint32_t rx_rear, rx_front, rx_base;
  uint32_t rx_indr_buf, rx_indr_buf_base, rx_indr_buf_size;
  uint32_t tx_rear, tx_front, tx_base;
  uint32_t q_len, q_elem_size, credit, mbox_addr;
  uint32_t ap_poll;
};

static uintptr_t gX1sSram = 0, gX1sMbox = 0, gX1sPmu = 0;
static uint32_t gX1sSeq = 0, gX1sStop = 70, gX1sGpuStop = 60;
extern "C" {
volatile int32_t x1s_heat_zone[4] = {-1, -1, -1, -1};   // last reading per zone, for a debugger / later sysctl
volatile uint32_t x1s_heat_reads = 0;
}

static uint32_t x1sRd (uintptr_t a) { return *reinterpret_cast<volatile uint32_t *> (a); }
static void x1sWr (uintptr_t a, uint32_t v) { *reinterpret_cast<volatile uint32_t *> (a) = v; }
#define X1S_CH(c, f) x1sRd ((c) + offsetof (X1sAcpmChannel, f))

static uintptr_t x1sMap (uint64_t pa, uint64_t size)
{
  IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress (pa, size, kIODirectionInOut);
  if (md == nullptr) { return 0; }
  IOMemoryMap *map = md->map (kIOMapAnywhere | kIOMapInhibitCache);   // kept for the life of the kernel
  md->release ();
  return map != nullptr ? static_cast<uintptr_t> (map->getVirtualAddress ()) : 0;
}

static uintptr_t x1sChannel (uint32_t id)
{
  uint32_t chans = x1sRd (gX1sSram + X1S_ACPM_INITDATA + 0x08), max = x1sRd (gX1sSram + X1S_ACPM_INITDATA + 0x18);
  if (chans == 0 || chans >= X1S_ACPM_SRAM_SIZE || max == 0 || max > 32) { return 0; }
  for (uint32_t i = 0; i < max; i++) {
    uintptr_t c = gX1sSram + chans + i * sizeof (X1sAcpmChannel);
    if (c + sizeof (X1sAcpmChannel) <= gX1sSram + X1S_ACPM_SRAM_SIZE && X1S_CH (c, id) == id) { return c; }
  }
  return 0;
}

// cmd[4] to the APM on a queue channel, its answer back in cmd: 0 or -1 (only the heat guard uses the APM here)
static int x1sAcpmSend (uint32_t id, uint32_t cmd[4])
{
  uintptr_t c = x1sChannel (id);
  if (c == 0) { return -1; }
  uint32_t len = X1S_CH (c, q_len), size = X1S_CH (c, q_elem_size);
  if (len == 0 || len > 64 || size < 16 || size > 64) { return -1; }
  uint32_t front = x1sRd (gX1sSram + X1S_CH (c, rx_front)), next = (front + 1) % len;
  for (unsigned w = 0; next == x1sRd (gX1sSram + X1S_CH (c, rx_rear)); w++) {
    if (w > 5000) { return -1; }                         // 50 ms
    IODelay (10);
  }
  gX1sSeq = gX1sSeq % 63 + 1;
  uint32_t s = gX1sSeq;
  cmd[0] = (cmd[0] & ~X1S_SEQ_MASK) | s << X1S_SEQ_SHIFT;
  for (uint32_t k = 0; k < size / 4; k++) { x1sWr (gX1sSram + X1S_CH (c, rx_base) + size * front + 4 * k, k < 4 ? cmd[k] : 0); }
  __asm__ volatile ("dsb sy" ::: "memory");
  x1sWr (gX1sSram + X1S_CH (c, rx_front), next);
  __asm__ volatile ("dsb sy" ::: "memory");
  x1sWr (gX1sMbox + 0x08, (1U << id) << 16);           // INTGR0
  for (unsigned w = 0; w <= 5000; w++) {
    uint32_t tf = x1sRd (gX1sSram + X1S_CH (c, tx_front)), rear = x1sRd (gX1sSram + X1S_CH (c, tx_rear));
    for (uint32_t i = rear; i != tf; i = (i + 1) % len) {
      uintptr_t e = gX1sSram + X1S_CH (c, tx_base) + size * i;
      if (((x1sRd (e) & X1S_SEQ_MASK) >> X1S_SEQ_SHIFT) != s) { continue; }
      for (uint32_t k = 0; k < 4; k++) { cmd[k] = x1sRd (e + 4 * k); }
      if (i != rear) {
        for (uint32_t k = 0; k < size / 4; k++) { x1sWr (e + 4 * k, x1sRd (gX1sSram + X1S_CH (c, tx_base) + size * rear + 4 * k)); }
      }
      rear = (rear + 1) % len;
      x1sWr (gX1sSram + X1S_CH (c, tx_rear), rear);
      if (rear == x1sRd (gX1sSram + X1S_CH (c, tx_front))) { x1sWr (gX1sMbox + 0x20, 1U << id); }   // INTCR1
      return 0;
    }
    IODelay (10);
  }
  return -1;
}

static int x1sZone (uint32_t zone)
{
  uint32_t cmd[4] = {0, 0x02U | zone << 16, 0, 0};
  if (x1sAcpmSend (9, cmd) != 0) { return -1; }
  int t = static_cast<int> (cmd[1] >> 24);
  return (t >= 5 && t <= 125) ? t : -1;
}

static int x1sPmicUpdate (uint32_t block, uint32_t reg, uint32_t val, uint32_t mask)
{
  uint32_t cmd[4] = {(block & 0xf) << 8 | (reg & 0xff), 2U | (val & 0xff) << 8 | (mask & 0xff) << 16, 0, 0};   // speedy 0, UPDATE
  int r = x1sAcpmSend (2, cmd);
  return r != 0 ? r : (((cmd[1] >> 24) & 0xff) != 0 ? -1 : 0);
}

static void __attribute__((noreturn)) x1sPowerOff (const char *why, int celsius)
{
  IOLog ("\n--- x1s heat guard: %s (%d C): powering the phone off ---\n", why, celsius);
  int wtsr = x1sPmicUpdate (0x02, 0x01, 0, (1U << 6) | (1U << 7));   // RTC block: WTSR_EN, SMPL_EN off
  int sub = x1sPmicUpdate (0x02, 0x03, 0, 1U << 3);                  // SUB_SMPL_EN off
  uint32_t stop[4] = {1U << 22, 0, 0, 0};                            // ACPM_PROTOCOL_STOP
  int apm = x1sAcpmSend (4, stop);
  IOLog ("x1s heat guard: WTSR/SMPL off %d, SUB_SMPL off %d, APM stop %d; PS_HOLD low\n", wtsr, sub, apm);
  IOSleep (100);
  __asm__ volatile ("msr daifset, #15" ::: "memory");
  __asm__ volatile ("dsb sy" ::: "memory");
  x1sWr (gX1sPmu + 0x30c, x1sRd (gX1sPmu + 0x30c) & ~(1U << 8));
  __asm__ volatile ("dsb sy" ::: "memory");
  for (;;) { __asm__ volatile ("wfi"); }                             // a USB cable keeps the power chip on: wait here
}

static void x1sHeatThread (void *, wait_result_t)
{
  int strikes = 0, gpuStrikes = 0, blindMs = 0, lastStep = 0;
  unsigned sinceLog = 10;
  for (;;) {
    int t[4], hottest = -1;
    for (uint32_t z = 0; z < 4; z++) {
      t[z] = x1sZone (z);
      x1s_heat_zone[z] = t[z];
      if (z < 3 && t[z] > hottest) { hottest = t[z]; }
    }
    x1s_heat_reads = x1s_heat_reads + 1;
    if (hottest < 0) {
      blindMs += 1000;
      if (blindMs >= 10000) { x1sPowerOff ("no temperature from the APM for 10 s", -1); }
    } else {
      blindMs = 0;
      int step = hottest >= 50 ? hottest / 5 : 0;
      if (++sinceLog >= 10 || step > lastStep) {
        IOLog ("x1s heat guard: BIG=%d MID=%d LITTLE=%d G3D=%d C\n", t[0], t[1], t[2], t[3]);
        sinceLog = 0;
      }
      lastStep = step;
      strikes = hottest >= static_cast<int> (gX1sStop) ? strikes + 1 : 0;
      gpuStrikes = t[3] >= static_cast<int> (gX1sGpuStop) ? gpuStrikes + 1 : 0;
      if (strikes >= 2) { x1sPowerOff ("a CPU zone at or above the limit", hottest); }
      if (gpuStrikes >= 2) { x1sPowerOff ("the GPU zone at or above its limit", t[3]); }
    }
    IOSleep (1000);
  }
}

static void x1sHeatStart (void)
{
  uint32_t v;
  if (PE_parse_boot_argn ("x1s_heat_stop", &v, sizeof (v)) && v >= 20 && v <= 70) { gX1sStop = v; }
  if (PE_parse_boot_argn ("x1s_heat_gpu_stop", &v, sizeof (v)) && v >= 20 && v <= 70) { gX1sGpuStop = v; }
  gX1sSram = x1sMap (X1S_ACPM_SRAM_PA, X1S_ACPM_SRAM_SIZE);
  gX1sMbox = x1sMap (X1S_ACPM_MBOX_PA, 0x1000);
  gX1sPmu = x1sMap (X1S_PMU_PA, 0x1000);
  if (gX1sSram == 0 || gX1sMbox == 0 || gX1sPmu == 0) {
    panic ("x1s heat guard: cannot map the APM or the PMU: no guard, no run");   // the owner's rule: no run without it
  }
  IOLog ("x1s heat guard: zones from the APM every 1 s; power off at %u C (GPU %u C)\n", gX1sStop, gX1sGpuStop);
  thread_t thread;
  if (kernel_thread_start (&x1sHeatThread, nullptr, &thread) != KERN_SUCCESS) { panic ("x1s heat guard: no thread"); }
  thread_deallocate (thread);
}
