#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H
/* Shared outer/inner deadline for multi-stage disc line alignment. */
#define PATH_DISC_LINE_TIMEOUT_MS 30000U
#ifndef PATH_STOP_AT_STAIR_LINE
#define PATH_STOP_AT_STAIR_LINE 0
#endif
/* Full mission enabled; host diagnostics may explicitly override to 0. */
#ifndef PATH_VISION_ENABLE
#define PATH_VISION_ENABLE 1
#endif
#if PATH_VISION_ENABLE != 0 && PATH_VISION_ENABLE != 1
#error "PATH_VISION_ENABLE must be 0 or 1"
#endif
#endif
