#ifndef TIMESPEC_H
#define TIMESPEC_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

struct timespec timespec_add(struct timespec ts1, struct timespec ts2);
struct timespec timespec_sub(struct timespec ts1, struct timespec ts2);
struct timespec timespec_mod(struct timespec ts1, struct timespec ts2);

bool timespec_eq(struct timespec ts1, struct timespec ts2);
bool timespec_gt(struct timespec ts1, struct timespec ts2);
bool timespec_ge(struct timespec ts1, struct timespec ts2);
bool timespec_lt(struct timespec ts1, struct timespec ts2);
bool timespec_le(struct timespec ts1, struct timespec ts2);

struct timespec timespec_normalise(struct timespec ts);

uint64_t timespec2ms(struct timespec ts);
uint64_t timespec2us(struct timespec ts);
#ifdef __cplusplus
}
#endif

#endif /* !DAN_TIMESPEC_H */
