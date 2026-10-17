#ifndef PATH_SESSION_H
#define PATH_SESSION_H
#include <stdint.h>
/* Call once during startup with HAL tick running, before the 5 ms control task.
 * Returns a cached per-boot random session; zero disables PATH (no weak fallback).
 */
uint32_t PathSession_Create(void);
#endif
