#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H
/* Shared outer/inner deadline for multi-stage disc line alignment. */
#define PATH_DISC_LINE_TIMEOUT_MS 30000U
/* Temporary chassis-only diagnostics; set to 1 to restore vision. */
#ifndef PATH_VISION_ENABLE
#define PATH_VISION_ENABLE 0
#endif
#if PATH_VISION_ENABLE != 0 && PATH_VISION_ENABLE != 1
#error "PATH_VISION_ENABLE must be 0 or 1"
#endif
#endif
