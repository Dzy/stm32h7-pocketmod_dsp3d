# LTDC video modes

`LTDC_VID_FORMAT` selects an entry from `LTDCSYNC[]` in `Core/Src/ltdc.c`.
The default is index 8, CTA-861 1920x1080p60 (VIC 16).

| Index | Standard | Mode | Pixel clock | H total | V total | Sync | AVI VIC |
|---:|---|---|---:|---:|---:|---|---:|
| 0 | VESA DMT | 640x480@72 | 31.5 MHz | 832 | 520 | -H/-V | disabled |
| 1 | VESA DMT | 640x480@75 | 31.5 MHz | 840 | 500 | -H/-V | disabled |
| 2 | VESA DMT | 800x600@72 | 50.0 MHz | 1040 | 666 | +H/+V | disabled |
| 3 | VESA DMT | 800x600@75 | 49.5 MHz | 1056 | 625 | +H/+V | disabled |
| 4 | VESA DMT | 800x600@85 | 56.25 MHz | 1048 | 631 | +H/+V | disabled |
| 5 | VESA DMT | 1024x768@70 | 75.0 MHz | 1328 | 806 | -H/-V | disabled |
| 6 | VESA DMT | 1024x768@75 | 78.75 MHz | 1312 | 800 | +H/+V | disabled |
| 7 | VESA DMT | 1280x1024@60 | 108.0 MHz | 1688 | 1066 | +H/+V | disabled |
| 8 | CTA-861 VIC 16 | 1920x1080p60 | 148.5 MHz | 2200 | 1125 | +H/+V | 16 |
| 9 | CTA-861 VIC 4 | 1280x720p60 | 74.25 MHz | 1650 | 750 | +H/+V | 4 |
| 10 | CTA-861 VIC 19 | 1280x720p50 | 74.25 MHz | 1980 | 750 | +H/+V | 19 |
| 11 | CTA-861 VIC 31 | 1920x1080p50 | 148.5 MHz | 2640 | 1125 | +H/+V | 31 |

Removed after hardware testing:

- old index 2: 640x480@85, left-edge artifact
- old index 3: 800x600@60, left-edge artifact
- old index 7: 1024x768@60, multiple artifacts
- old filtered index 10: 1920x1080p30, Lenovo G24e-20 reports `Input Not Supported`
- old filtered index 13: 1920x1080p24, below the Lenovo G24e-20 vertical-frequency range

For CTA modes, TDA9983B packet slot IF2 now transmits an AVI InfoFrame with:

- RGB input format
- 16:9 picture aspect ratio
- the mode-specific CTA VIC
- full-range RGB quantization
- no pixel repetition

VESA DMT modes use VIC 0 and disable AVI IF2. USB host, PocketMod/I2S audio and HDMI audio are retained.
