#ifndef PATH_DESTACK_H
#define PATH_DESTACK_H
#include "path_mission.h"
enum { DS_ENTRY=31, DS_MAP=32, DS_THIRD, DS_POSE, DS_SCAN, DS_CENTER, DS_STOP_PICK,
       DS_PICK, DS_FOURTH, DS_PLACE, DS_RETURN, DS_STOP_ROW, DS_BALL_MOVE, DS_BALL };
bool PathDestack_Tick(PathMission *m,uint32_t now,const PathInput *in);
bool PathDestack_Advance(PathMission *m,uint32_t now);
#endif
