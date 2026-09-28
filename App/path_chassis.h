#ifndef PATH_CHASSIS_H
#define PATH_CHASSIS_H
#include "path_mission.h"
bool PathLine_Align(PathMission *mission, uint32_t now, const PathInput *input,
                    uint32_t timeout, float lateral);
bool PathLine_Aligned(uint8_t gray);
bool PathLine_AlignFour(PathMission *mission, uint32_t now, const PathInput *input);
bool PathHeading_Ready(PathMission *mission, uint32_t now, const PathInput *input);
void PathChassis_Tick(PathMission *mission, uint32_t now, const PathInput *input);
#endif
