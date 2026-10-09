#ifndef PATH_DESTACK_H
#define PATH_DESTACK_H
#include "path_mission.h"
/* Phases are visible in telemetry. Ball unloading keeps its original phase values. */
enum {
    DESTACK_POSE=32, DESTACK_CHECK, DESTACK_PICK, DESTACK_TO_FOURTH,
    DESTACK_PLACE, DESTACK_RETURN, DESTACK_NEXT, DESTACK_HOME
};
bool PathDestack_Tick(PathMission *m, uint32_t now, const PathInput *in);
bool PathDestack_Advance(PathMission *m, uint32_t now);
#endif
