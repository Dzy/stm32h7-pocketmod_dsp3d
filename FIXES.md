# GCC 15 / STM32H7 build fixes

This package updates the original project for current Arm GNU C compilers.

Key changes:

- selects `LTDC_VID_FORMAT` exactly once instead of redefining it repeatedly;
- adds missing function declarations, including `tda_init()`;
- fixes DMA2D, I2S, LTDC CLUT, SDRAM and framebuffer pointer/address types;
- uses bounded string formatting with matching integer format types;
- removes obsolete external-inline declarations that caused missing or conflicting dsp3D symbols;
- fixes low-level framebuffer and depth-buffer address conversions;
- fixes TDA19988 I2C array-pointer arguments;
- fixes the HID report writer to use its calculated byte offset;
- uses explicit GNU C11 mode in the Makefile;
- corrects source paths in the Makefile.

Verification performed:

- all 61 C translation units were checked and compiled for
  `arm-none-eabi`, Cortex-M7, Thumb, hard-float, FPv5-D16;
- the resulting ARM EABI5 objects were combined with a relocatable link;
- no C compiler diagnostics remained in that verification pass.

Build normally with:

```sh
make clean && make
```

## Cache and double-buffering update

This revision also changes the framebuffer ownership and cache policy:

- the complete 32 MiB SDRAM remains write-back cacheable for ordinary CPU data;
- MPU region 1 overlays the first 8 MiB as Normal, non-cacheable memory for the
  two 4 MiB framebuffer slots;
- the depth buffer at `0xC0800000` remains in the cacheable SDRAM region;
- both framebuffers are cleared with DMA2D before LTDC is enabled;
- later clears also use the existing RGB565-as-two-L8-bytes DMA2D technique;
- LTDC address reload is requested for vertical blanking and handled
  asynchronously;
- software does not exchange front/back ownership until the LTDC reload
  callback confirms that the new front buffer is active;
- USB host processing continues while a vertical-blank reload is pending;
- the old front buffer is cleared only after it has become the back buffer;
- framebuffer writes are completed with barriers before presenting;
- low-level point drawing now rejects coordinates outside the active viewport,
  preventing writes outside the framebuffer.

The active layout is defined in `Core/Inc/main.h`:

```c
#define FRAMEBUFFER0_ADDRESS 0xC0000000U
#define FRAMEBUFFER1_ADDRESS 0xC0400000U
#define DEPTH_BUFFER_ADDRESS 0xC0800000U
```

## Video mode update

- Removed old modes 2, 3 and 7 after target-hardware artifact testing.
- Old mode 8 (1024x768@70) moved to index 5; it remains available as the known-good VESA fallback.
- Added CTA-861 720p60, 720p50, 1080p50 and 1080p60 modes.
- Removed CTA 1080p30 and 1080p24 after the Lenovo G24e-20 rejected/bounded them below its supported vertical-frequency range.
- The default is now index 8: CTA VIC 16, 1920x1080p60.
- Added TDA9983B AVI InfoFrame generation in IF2 with VIC, 16:9 aspect and full-range RGB metadata.
- Existing CTA-861 1080p60 was retained at new index 8.
- USB and HDMI audio paths were not modified.
