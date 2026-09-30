#ifndef PATH_WAREHOUSE_H
#define PATH_WAREHOUSE_H
#include "path_mission.h"
uint8_t PathWarehouse_Code(const PathMission *m);
void PathWarehouse_Tick(PathMission *m, uint32_t now, const PathInput *in);
#endif
