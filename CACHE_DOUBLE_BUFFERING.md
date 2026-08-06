# STM32H7 framebuffer ownership

The two L8 framebuffers occupy fixed 4 MiB slots:

- front slot: `0xC0000000`
- back slot: `0xC0400000`
- cached depth-buffer area begins at `0xC0800000`

The first 8 MiB of SDRAM is configured as Normal, non-cacheable memory. This
keeps CPU, DMA2D and LTDC coherent without cleaning or invalidating the full
frame on every presentation. The rest of SDRAM remains write-back cacheable.

Presentation is non-blocking:

1. CPU and DMA2D render only into `bbuffer`.
2. `Framebuffer_QueuePresent()` places that address in the LTDC shadow register
   and requests a vertical-blank reload.
3. The main loop continues calling `MX_USB_HOST_Process()` while the reload is
   pending.
4. `HAL_LTDC_ReloadEventCallback()` only marks completion.
5. The main loop then exchanges `fbuffer` and `bbuffer`.
6. The old front buffer is cleared by DMA2D and becomes the next draw buffer.

The software pointers are therefore never swapped before LTDC confirms the
hardware address change.
