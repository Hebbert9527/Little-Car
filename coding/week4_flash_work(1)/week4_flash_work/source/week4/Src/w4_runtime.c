#include "w4_runtime.h"
#include "w4_period.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include <string.h>
typedef struct {
    W4TaskStats stats;
    uint64_t previous,start;
    uint32_t release;
    bool seen,active;
} Record;
static Record records[W4_TASK_COUNT];
static W4HeapStats heap;
static void add(W4Range *r,uint64_t n)
{
    if (!r->count || n<r->min) r->min=n;
    if (n>r->max) r->max=n;
    r->total+=n; ++r->count;
}
void W4Runtime_Init(void)
{
    memset(records,0,sizeof records);
    for (unsigned i=0;i<W4_TASK_COUNT;++i) records[i].stats.stack_free_min=W4_TASKS[i].attr.stack_size;
    heap=(W4HeapStats){0,0,configTOTAL_HEAP_SIZE};
}
void W4Runtime_Begin(W4TaskId id,uint32_t release)
{
    if ((unsigned)id>=W4_TASK_COUNT) return;
    uint64_t now=W4Clock_Cycles();
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    Record *r=&records[id];
    if (r->seen) {
        uint64_t dt=now-r->previous; add(&r->stats.period_cycles,dt);
        if (W4_TASKS[id].period_ticks) {
            uint64_t ideal=(uint64_t)W4Clock_Hz()*W4_TASKS[id].period_ticks/1000;
            add(&r->stats.jitter_cycles,dt>ideal?dt-ideal:ideal-dt);
        }
    }
    ++r->stats.wakes; r->seen=true; r->active=true;
    r->previous=now; r->start=now; r->release=release;
    __set_PRIMASK(mask);
}
void W4Runtime_End(W4TaskId id,uint32_t deadline)
{
    if ((unsigned)id>=W4_TASK_COUNT) return;
    uint64_t now=W4Clock_Cycles(); uint32_t tick=osKernelGetTickCount();
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    Record *r=&records[id];
    if (r->active) {
        add(&r->stats.elapsed_cycles,now-r->start);
        if (deadline && W4Tick_After(tick,r->release+deadline)) ++r->stats.deadline_misses;
        r->active=false;
    }
    __set_PRIMASK(mask);
}
bool W4Runtime_Get(W4TaskId id,W4TaskStats *out)
{
    if ((unsigned)id>=W4_TASK_COUNT || !out) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=records[id].stats;
    __set_PRIMASK(mask); return true;
}
void W4Runtime_CollectResources(void)
{
    for (unsigned i=0;i<W4_TASK_COUNT;++i) {
        uint32_t free=W4Threads[i]?osThreadGetStackSpace(W4Threads[i]):0;
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if (free<records[i].stats.stack_free_min) records[i].stats.stack_free_min=free;
        records[i].stats.stack_used_max=W4_TASKS[i].attr.stack_size-records[i].stats.stack_free_min;
        __set_PRIMASK(mask);
    }
    W4HeapStats value={xPortGetFreeHeapSize(),xPortGetMinimumEverFreeHeapSize(),configTOTAL_HEAP_SIZE};
    uint32_t mask=__get_PRIMASK(); __disable_irq(); heap=value; __set_PRIMASK(mask);
}
W4HeapStats W4Runtime_GetHeap(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    W4HeapStats result=heap; __set_PRIMASK(mask); return result;
}
