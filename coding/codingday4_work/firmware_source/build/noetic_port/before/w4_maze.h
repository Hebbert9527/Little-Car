#ifndef W4_MAZE_H
#define W4_MAZE_H
#include "w4_nav.h"
/* Path task owns mapping/planning. Control owns stopping and leg completion. */
void W4Maze_Point(float angle,float mm,unsigned quality,bool start,uint32_t now);
void W4Maze_Poll(const W4NavSnapshot *nav);
void W4Maze_Begin(uint32_t id);
void W4Maze_Reset(void);
bool W4Maze_Active(uint32_t id);
bool W4Maze_Fresh(uint32_t now);
bool W4Maze_LegBlocked(void);
bool W4Maze_FinalLeg(void);
void W4Maze_NextLeg(void);
#endif
