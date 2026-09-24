#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H
/* Full mission enabled; set to 0 for temporary chassis-only diagnostics. */
#ifndef PATH_VISION_ENABLE
#define PATH_VISION_ENABLE 1
#endif
#if PATH_VISION_ENABLE != 0 && PATH_VISION_ENABLE != 1
#error "PATH_VISION_ENABLE must be 0 or 1"
#endif
#endif
