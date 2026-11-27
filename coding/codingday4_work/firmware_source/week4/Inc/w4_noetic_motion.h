#ifndef W4_NOETIC_MOTION_H
#define W4_NOETIC_MOTION_H
#include "w4_nav.h"
#include "planner_config.h"
typedef struct { bool valid,reached; float v,w; Point2D lookahead; unsigned index; } W4NoeticCommand;
void W4Noetic_Reset(void);
void W4Noetic_Load(const Point2D *path,unsigned count,const W4Pose *pose);
void W4Noetic_Swap(const Point2D *path,unsigned count,const W4Pose *pose);
W4NoeticCommand W4Noetic_Step(const W4Pose *pose,float speed,float dt);
unsigned W4Noetic_Index(void);
bool W4Noetic_OffPath(void);
#endif
