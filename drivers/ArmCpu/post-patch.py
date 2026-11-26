"""Add a GICv2 (GIC-400) mode to UniXNU's ArmCpu.cpp for the Galaxy S20 (Exynos 990). usage: gicv2_patch.py ArmCpu.cpp
(run on the driver AFTER the build has applied patches/drivers/ArmCpu-*.patch)"""
import sys

p = sys.argv[1]
s = open(p, newline="").read().replace("\r\n", "\n")


import re


def _asm(t):
    # match the build's ArmCpu-arm64-assembly.patch: no ';' inside inline assembly, explicit newlines instead
    return re.sub(r'(__asm__ volatile \("[^"]*")', lambda m: m.group(1).replace('; ', r'\n\t'), t)


def rep(a, b):
    global s
    a, b = _asm(a), _asm(b)
    assert s.count(a) == 1, (a[:70], s.count(a))
    s = s.replace(a, b)


rep("static bool gRemoteGic = false;",
    """// x1s (Exynos 990): GIC-400 (GICv2). Distributor + memory-mapped CPU interface instead of the GICv3 redistributors and
// ICC_* system registers (undefined instructions on a GICv2 system). Chosen from the DT compatible of /arm-io/intc.
static bool gGicV2 = false;
static uintptr_t gCpuIf = 0;                 // GICC frame (virtual)
static uint8_t gV2BootMask = 1;              // the boot core's CPU interface bit (its banked ITARGETSR0 byte)
static uint8_t gV2Mask[ARMCPU_SMP_SLOTS];    // each logical core's CPU interface bit, for GICD_SGIR
static bool gRemoteGic = false;""")

rep("""  bool valid = (ranges != nullptr) && (ranges->getLength () == sizeof (bus)) &&
               (reg != nullptr) && (reg->getLength () == sizeof (regions)) &&
               (compatible != nullptr) && (compatible->getLength () == 11) &&
               (memcmp (compatible->getBytesNoCopy (), "arm,gic-v3", 11) == 0);""",
    """  bool v3 = (compatible != nullptr) && (compatible->getLength () == 11) &&
            (memcmp (compatible->getBytesNoCopy (), "arm,gic-v3", 11) == 0);
  bool v2 = (compatible != nullptr) &&
            (((compatible->getLength () == 12) && (memcmp (compatible->getBytesNoCopy (), "arm,gic-400", 12) == 0)) ||
             ((compatible->getLength () == 19) && (memcmp (compatible->getBytesNoCopy (), "arm,cortex-a15-gic", 19) == 0)));
  bool valid = (ranges != nullptr) && (ranges->getLength () == sizeof (bus)) &&
               (reg != nullptr) && (reg->getLength () == sizeof (regions)) && (v3 || v2);
  gGicV2 = v2;""")

rep("""  if (!valid || (regions[1] < 0x10000) || (regions[3] < 0x20000)) {
    return false;
  }""", """  if (!valid || (v3 && ((regions[1] < 0x10000) || (regions[3] < 0x20000))) ||
      (v2 && ((regions[1] < 0x1000) || (regions[3] < 0x2000)))) {
    return false;
  }""")

rep("""  dist = static_cast<uintptr_t> (distributor->getVirtualAddress ());
  if (((read32 (dist, 0xffe8) >> 4) & 15) != 3) {
    return false;
  }""", """  dist = static_cast<uintptr_t> (distributor->getVirtualAddress ());
  if (v2) {
    // GIC-400: PIDR2 (0xfe8) ArchRev 2; the CPU interface is the second "reg" region.
    if (((read32 (dist, 0xfe8) >> 4) & 15) != 2) {
      return false;
    }
    gCpuIf = static_cast<uintptr_t> (redistributors->getVirtualAddress ());
  } else if (((read32 (dist, 0xffe8) >> 4) & 15) != 3) {
    return false;
  }""")

rep("""bool ArmCpu::initDistributor (void)
{
  uint32_t ctl""", """bool ArmCpu::initDistributor (void)
{
  if (gGicV2) {
    write32 (dist, 0, 0);                                    // GICD_CTLR off while it is set up
    unsigned interrupts = ((read32 (dist, 4) & 31) + 1) * 32;
    for (unsigned id = 32; id < interrupts && id < 1020; id += 32) {
      write32 (dist, 0x180 + id / 8, 0xffffffff);            // ICENABLER: every SPI off until a driver attaches it
      write32 (dist, 0x280 + id / 8, 0xffffffff);            // ICPENDR
    }
    for (unsigned id = 32; id < interrupts && id < 1020; id += 4) {
      write32 (dist, 0x400 + id, 0xa0a0a0a0);                // IPRIORITYR
    }
    return true;
  }
  uint32_t ctl""")

rep("""  uintptr_t frames = redistBase;
  // local:""", """  if (gGicV2) {
    if (!local) { return false; }                            // GICv2: a core sets up only its own banked registers
    __asm__ volatile ("mrs %0, mpidr_el1" : "=r" (affinity));
    write32 (dist, 0x180, 0xffffffff);                       // ICENABLER0 (banked per core)
    write32 (dist, 0x280, 0xffffffff);                       // ICPENDR0
    for (unsigned offset = 0x400; offset < 0x420; offset += 4) {
      write32 (dist, offset, 0x80808080);                    // IPRIORITYR0..7: SGIs and PPIs
    }
    __asm__ volatile ("dsb sy" ::: "memory");
    return true;
  }
  uintptr_t frames = redistBase;
  // local:""")

rep("""void ArmCpu::enableGic (void)
{
  uint64_t value;""", """void ArmCpu::enableGic (void)
{
  if (gGicV2) {
    uint8_t mask = static_cast<uint8_t> (read32 (dist, 0x800) & 0xff);   // banked ITARGETSR0: this core's own bit
    if (mask == 0) { mask = 1; }
    gV2Mask[logicalId & (ARMCPU_SMP_SLOTS - 1)] = mask;
    if (!secondary) { gV2BootMask = mask; gBootAffinity = affinity & 0xff00ffffffULL; }
    write32 (gCpuIf, 0x4, 0xff);                             // GICC_PMR: let everything through
    write32 (gCpuIf, 0x8, 0);                                // GICC_BPR
    write32 (dist, 0x100, kSgiPpiEnable);                    // ISENABLER0: SGIs and the virtual timer PPI 27
    write32 (dist, 0, read32 (dist, 0) | 1U);                // GICD_CTLR enable
    write32 (gCpuIf, 0x0, 1U);                               // GICC_CTLR enable (EOImode 0)
    __asm__ volatile ("dsb sy; isb" ::: "memory");
    if (!secondary) {
      armcpu_gic_boot[0] = read32 (dist, 0x80);
      armcpu_gic_boot[1] = read32 (dist, 0x100);
      armcpu_gic_boot[2] = read32 (dist, 0);
      armcpu_gic_boot[3] = read32 (dist, 4);
      armcpu_gic_boot[4] = mask;
      armcpu_gic_boot[5] = read32 (gCpuIf, 0x4);
      armcpu_gic_boot[6] = read32 (gCpuIf, 0x0);
      armcpu_gic_boot[7] = 2;                                // GIC version
    }
    armcpu_gic_ready = 1;
    return;
  }
  uint64_t value;""")

rep("""    uint64_t id;
    __asm__ volatile ("mrs %0, icc_iar1_el1" : "=r" (id));""", """    uint64_t id;
    if (gGicV2) { id = read32 (gCpuIf, 0xc) & 0x3ff; }       // GICC_IAR (no source CPU bits: SGIs come from GICD_SGIR)
    else { __asm__ volatile ("mrs %0, icc_iar1_el1" : "=r" (id)); }""")

rep("""    __asm__ volatile ("msr icc_eoir1_el1, %0; isb" :: "r" (id) : "memory");
  }
}""", """    if (gGicV2) { write32 (gCpuIf, 0x10, static_cast<uint32_t> (id)); __asm__ volatile ("isb" ::: "memory"); }
    else { __asm__ volatile ("msr icc_eoir1_el1, %0; isb" :: "r" (id) : "memory"); }
  }
}""")

rep("""  if (gSpiIpi) { spiIpiSend (gDist, peer->logicalId & 7); return; }
  __asm__ volatile ("dsb ishst; msr icc_sgi1r_el1, %0; isb" :: "r" (sgi) : "memory");
}""", """  if (gSpiIpi) { spiIpiSend (gDist, peer->logicalId & 7); return; }
  if (gGicV2) {
    __asm__ volatile ("dsb ishst" ::: "memory");
    write32 (gDist, 0xf00, static_cast<uint32_t> (gV2Mask[peer->logicalId & (ARMCPU_SMP_SLOTS - 1)]) << 16);   // GICD_SGIR, SGI 0
    return;
  }
  __asm__ volatile ("dsb ishst; msr icc_sgi1r_el1, %0; isb" :: "r" (sgi) : "memory");
}""")

rep("""  uint64_t value = 0;
  __asm__ volatile ("msr icc_igrpen1_el1, %0; dsb sy; isb" :: "r" (value) : "memory");
}""", """  if (gGicV2) { write32 (gCpuIf, 0x0, 0); __asm__ volatile ("dsb sy; isb" ::: "memory"); return; }
  uint64_t value = 0;
  __asm__ volatile ("msr icc_igrpen1_el1, %0; dsb sy; isb" :: "r" (value) : "memory");
}""")

rep("""  *reinterpret_cast<volatile uint64_t *> (gDist + 0x6000 + 8 * intid) = gBootAffinity;""",
    """  if (gGicV2) { *reinterpret_cast<volatile uint8_t *> (gDist + 0x800 + intid) = gV2BootMask; }   // ITARGETSR
  else { *reinterpret_cast<volatile uint64_t *> (gDist + 0x6000 + 8 * intid) = gBootAffinity; }""")

rep("""  if (router != nullptr) { *router = *reinterpret_cast<volatile uint64_t *> (gDist + 0x6000 + 8 * intid); }""",
    """  if (router != nullptr) {
    *router = gGicV2 ? *reinterpret_cast<volatile uint8_t *> (gDist + 0x800 + intid)
                     : *reinterpret_cast<volatile uint64_t *> (gDist + 0x6000 + 8 * intid);
  }""")

rep("""extern "C" void armcpu_sgi_selfprobe (uint64_t *out)
{""", """extern "C" void armcpu_sgi_selfprobe (uint64_t *out)
{
  if (gGicV2) { for (unsigned i = 0; i < 10; i++) { out[i] = 0; } return; }   // uses ICC_* registers: GICv3 only""")

rep("""extern "C" void armcpu_ipi_selftest (uint64_t aff)
{""", """extern "C" void armcpu_ipi_selftest (uint64_t aff)
{
  if (gGicV2) { return; }""")

rep("""  for (unsigned i = 0; i < 8; i++) { out[i] = 0xdeadbeef; }
  if (gDist == 0 || gFrames == 0)""", """  for (unsigned i = 0; i < 8; i++) { out[i] = 0xdeadbeef; }
  if (gGicV2) { armcpu_gic_diag_stage = 0xe; return; }
  if (gDist == 0 || gFrames == 0)""")

# ---- x1s heat guard (heatguard.inc beside this script): included at the end, started by the boot CPU ----
import os
rep("#include <pexpert/pexpert.h>\n", "#include <pexpert/pexpert.h>\n#include <IOKit/IOLib.h>\n#include <stddef.h>\n")
rep("static bool gGicV2 = false;", "static void x1sHeatStart (void);\nstatic bool gGicV2 = false;")
rep("""  registerService ();
  __atomic_store_n (&gBootRegistered, 1U, __ATOMIC_RELEASE);""", """  registerService ();
  x1sHeatStart ();                                           // x1s: the heat guard, before anything else can heat the phone
  __atomic_store_n (&gBootRegistered, 1U, __ATOMIC_RELEASE);""")
s += open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'heatguard.inc')).read()


# ---- x1s bring-up bars in ArmCpu (x1s_bar() is in the kernel: patches/xnu-12377/zzz-x1s-zzzzzzzz-boot-markers) ----
rep("bool ArmCpu::start (IOService *provider)\n{\n", "extern \"C\" void x1s_bar (unsigned, uint32_t);\nbool ArmCpu::start (IOService *provider)\n{\n  x1s_bar (27, 0xFFFFFFFF);                                  // white: ArmCpu::start\n")
rep("    if ((!skipGic && !prepareGic ()) || !IOCPU::start (provider)) { return false; }",
    "    if (!skipGic && !prepareGic ()) { x1s_bar (28, gGicV2 ? 0xFFFF8000 : 0xFFFF0000); return false; }   // orange/red: GIC setup failed (v2 / not v2)\n    x1s_bar (28, 0xFF00FF00);                                // green: GIC set up\n    if (!IOCPU::start (provider)) { x1s_bar (29, 0xFFFF0000); return false; }")

open(p, 'w', newline='').write(s)
print('patched', s.count('gGicV2'), 'uses')
