#ifndef W4_RUNTIME_H
#define W4_RUNTIME_H
#include "w4_rtos.h"
#include <stdint.h>
#include <stdbool.h>
typedef struct { uint64_t min,max,total; uint32_t count; } W4Range;
typedef struct {
    uint32_t wakes,deadline_misses,stack_free_min,stack_used_max;
    W4Range period_cycles,jitter_cycles,elapsed_cycles;
} W4TaskStats;
typedef struct { uint32_t free_bytes,min_free_bytes,total_bytes; } W4HeapStats;
void W4Clock_Init(void);
void W4Clock_OnHalTick(void); /* TIM6 ISR only, including before kernel start */
uint64_t W4Clock_Cycles(void);
uint32_t W4Clock_Hz(void);
void W4Runtime_Init(void);
/* One writer per task. Bracket useful work, excluding the next event/period wait. */
void W4Runtime_Begin(W4TaskId id,uint32_t scheduled_release);
void W4Runtime_End(W4TaskId id,uint32_t deadline_ticks);
bool W4Runtime_Get(W4TaskId id,W4TaskStats *out);
/* Low-priority task, about once per second, outside measured work. */
void W4Runtime_CollectResources(void);
W4HeapStats W4Runtime_GetHeap(void);
#endif
