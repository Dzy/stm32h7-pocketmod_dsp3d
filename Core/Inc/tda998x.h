#ifndef TDA998X_H
#define TDA998X_H

#ifdef __cplusplus
extern "C" {
#endif

/* The transmitter implementation keeps the original linker symbol tda_init.
 * Calls through this header also program the mode-specific AVI InfoFrame after
 * the existing video and HDMI-audio initialization has completed.
 */
void tda_init_hw(void) __asm__("tda_init");
void tda_apply_avi_infoframe(void);
#define tda_init() do { tda_init_hw(); tda_apply_avi_infoframe(); } while (0)

#ifdef __cplusplus
}
#endif

#endif /* TDA998X_H */
