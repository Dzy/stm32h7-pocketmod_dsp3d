# Video-mode and AVI InfoFrame update

This branch updates the LTDC/TDA9983B video path while retaining the existing USB host, PocketMod/I2S audio and HDMI-audio initialization.

Changes:

- corrected the retained VESA DMT mode timings and added per-mode HSYNC/VSYNC polarities;
- removed modes that produced artifacts on target hardware;
- removed 1080p30 and 1080p24 modes rejected by the Lenovo G24e-20 test monitor;
- retained CTA-861 720p50, 720p60, 1080p50 and 1080p60 modes;
- selected CTA VIC 16, 1920x1080p60, as default index 8;
- added TDA9983B AVI InfoFrame programming in IF2 with RGB, 16:9, mode-specific VIC, full-range quantization and no pixel repetition;
- VESA modes use VIC 0 and disable AVI IF2.

The existing `tda_init()` implementation is called first. AVI metadata is programmed afterwards so the original HDMI video and audio initialization remains intact.
