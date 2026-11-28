#include "benchmark.h"
#include "main.h"

void benchmark_init(void)
{
    /* Enable trace hardware */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset cycle counter */
    DWT->CYCCNT = 0;

    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t benchmark_get_cycles(void)
{
    return DWT->CYCCNT;
}

float benchmark_cycles_to_us(uint32_t cycles)
{
    return ((float)cycles * 1000000.0f)
           / (float)SystemCoreClock;
}
