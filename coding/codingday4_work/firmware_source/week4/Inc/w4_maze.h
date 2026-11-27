#ifndef W4_MAZE_H
#define W4_MAZE_H
#include "w4_nav.h"
#include "w4_noetic_motion.h"
/* Path task owns mapping/full-route planning; control owns tracking/stopping. */
void W4Maze_Point(float angle,float mm,unsigned quality,bool start,uint32_t now);
void W4Maze_Poll(const W4NavSnapshot *nav);
void W4Maze_Begin(uint32_t id,bool returning);
void W4Maze_Reset(void);
bool W4Maze_Active(uint32_t id);
bool W4Maze_Returning(void);
bool W4Maze_Fresh(uint32_t now);
bool W4Maze_ImmediateBlocked(void);
bool W4Maze_FinalLeg(void);
void W4Maze_RequestReplan(void);
bool W4Maze_HasNewRoute(uint32_t id,uint32_t loaded_seq);
bool W4Maze_LoadRoute(uint32_t id,const W4Pose *pose,bool hot_swap,uint32_t *loaded_seq);
W4NoeticCommand W4Maze_Track(const W4Pose *pose,float speed,float dt);
unsigned W4Maze_RouteCount(void);
unsigned W4Maze_TrailCount(void);
#endif
