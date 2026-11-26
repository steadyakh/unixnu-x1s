# unixnu-x1s

UniXNU device repository for the **Samsung Galaxy S20 5G** (SM-G981B, x1s, Exynos 990), targeting iOS 27.2 with the
iPhone 14 Pro (iPhone15,2 / D73) identity. Experimental: nothing boots yet.

- Kernel: [UniXNU](https://github.com/steadyakh/UniXNU) xnu-12377, built on a GitHub macOS runner
  (`macOS build` workflow, device repository `steadyakh/unixnu-x1s`, defconfig `x1s_defconfig`).
- Userland: Apple's arm64e iOS 27 is lowered to arm64 on the PC, since the S20's cores have no pointer
  authentication (method from ios-guacamole, OnePlus 7 Pro).
- Boot: Mu-Silicium UEFI (x1sPkg) from the boot partition.

No Apple files, Samsung firmware or device dumps go into this repository.
