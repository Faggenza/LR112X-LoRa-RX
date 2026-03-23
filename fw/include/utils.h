#ifndef UTILS_H
#define UTILS_H

#include "main.h"

#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
void log_payload(const uint8_t *payload, uint8_t len);
void uart_log(const char *fmt, ...);

#endif /* UTILS_H */