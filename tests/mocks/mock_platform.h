#ifndef MOCK_PLATFORM_H_
#define MOCK_PLATFORM_H_

#include <stdint.h>

extern uint32_t mock_delay_total_ms;
extern uint32_t mock_delay_call_count;

void mock_platform_reset(void);

#endif
