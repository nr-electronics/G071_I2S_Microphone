#ifndef __debug_H
#define __debug_H

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
 extern "C" {
#endif

extern bool debug_show_time_stamp;

void show_debug_timestamp(bool is_show);

void debug_out(const char *__fmt, ...);
void debug_dump(const char *msg, uint8_t * buffer, size_t size);


#ifndef USAGE_DEBUG_OUT
# define debug_out(...)
# define debug_dump(...)
#endif


#ifdef __cplusplus
}
#endif
#endif /* __debug_H */
