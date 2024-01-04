/*
 * soft_timer.h
 */

#ifndef INC_SOFT_TIMER_H_
#define INC_SOFT_TIMER_H_

#include <stdint.h>

typedef uint32_t soft_timer_t, *soft_timer_p;
#define get_tick_timer()						HAL_GetTick()
#define declare_timer(__timer__)				soft_timer_t __timer__=0
#define reset_timer(__timer__)					__timer__=get_tick_timer()
#define check_timeout(__timer__, __tm_out__)	((__timer__+__tm_out__)<=get_tick_timer())
#define get_timer_ms(__timer__)					(get_tick_timer()-__timer__)

#endif /* INC_SOFT_TIMER_H_ */
