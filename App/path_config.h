#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H
/* Temporary chassis-only run: set to 1 to restore RDK/vision/arm/RFID tasks. */
#ifndef PATH_VISION_ENABLE
#define PATH_VISION_ENABLE 0
#endif
#if PATH_VISION_ENABLE != 0 && PATH_VISION_ENABLE != 1
#error "PATH_VISION_ENABLE must be 0 or 1"
#endif
#endif
