#include "platform.h"
#include "main.h"

void platform_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
