#ifndef SIM_LOG_H
#define SIM_LOG_H

#include <stdio.h>

#define LOG_CAT_SERIAL 0

void sim_log_info(int category, const char *fmt, ...);
void sim_log_warn(int category, const char *fmt, ...);
void sim_log_error(int category, const char *fmt, ...);

#define SIM_LOG_INFO(cat, fmt, ...) sim_log_info(cat, fmt, ##__VA_ARGS__)
#define SIM_LOG_WARN(cat, fmt, ...) sim_log_warn(cat, fmt, ##__VA_ARGS__)
#define SIM_LOG_ERROR(cat, fmt, ...) sim_log_error(cat, fmt, ##__VA_ARGS__)

#endif

