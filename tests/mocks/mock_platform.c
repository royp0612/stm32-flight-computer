#include "platform.h"
#include "mock_platform.h"

uint32_t mock_delay_total_ms = 0;
uint32_t mock_delay_call_count = 0;

void mock_platform_reset(void)
{
    mock_delay_total_ms = 0;
    mock_delay_call_count = 0;
}

void platform_delay_ms(uint32_t ms)
{
    mock_delay_total_ms += ms;
    mock_delay_call_count++;
}
