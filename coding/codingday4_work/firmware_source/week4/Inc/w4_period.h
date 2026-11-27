#ifndef W4_PERIOD_H
#define W4_PERIOD_H
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
typedef struct { uint32_t release, period, skipped; } W4Period;
/* Comparisons valid when timestamps are less than 2^31 ticks apart. */
bool W4Tick_After(uint32_t a,uint32_t b);
uint32_t W4Period_Next(uint32_t previous,uint32_t now,uint32_t period,uint32_t *skipped);
bool W4Period_Init(W4Period *p,uint32_t period);
osStatus_t W4Period_Wait(W4Period *p);
/* TX: deadline for the next 50 ms publication; DMA events may wake sooner. */
uint32_t W4Period_Remaining(uint32_t deadline,uint32_t now);
#endif
