/* Functions for working with timespec structures
 * Written by Daniel Collins (2017)
 * timespec_mod by Alex Forencich (2019)
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 */

/** \file timespec.c
 *  \brief Functions for working with timespec structures.
 *
 * This library aims to provide a comprehensive set of functions with
 * well-defined behaviour that handle all edge cases (e.g. negative values) in
 * a sensible manner.
 *
 * Negative values are allowed in the tv_sec and/or tv_usec field of timespec
 * structures, tv_usec is always relative to tv_sec, so mixing positive and
 * negative values will produce consistent results:
 *
 * <PRE>
 * { tv_sec = 1,  tv_nsec = 500000000  } ==  1.5 seconds
 * { tv_sec = 1,  tv_nsec = 0          } ==  1.0 seconds
 * { tv_sec = 1,  tv_nsec = -500000000 } ==  0.5 seconds
 * { tv_sec = 0,  tv_nsec = 500000000  } ==  0.5 seconds
 * { tv_sec = 0,  tv_nsec = 0          } ==  0.0 seconds
 * { tv_sec = 0,  tv_nsec = -500000000 } == -0.5 seconds
 * { tv_sec = -1, tv_nsec = 500000000  } == -0.5 seconds
 * { tv_sec = -1, tv_nsec = 0          } == -1.0 seconds
 * { tv_sec = -1, tv_nsec = -500000000 } == -1.5 seconds
 * </PRE>
 *
 * Furthermore, any timespec structure processed or returned by library functions
 * is normalised according to the rules in timespec_normalise().
 */

#include <limits.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>

#include "timespec.h"

#define NSEC_PER_SEC 1000000000

/** \fn struct timespec timespec_add(struct timespec ts1, struct timespec ts2)
 *  \brief Returns the result of adding two timespec structures.
 */
struct timespec timespec_add(struct timespec ts1, struct timespec ts2)
{
    /* Normalise inputs to prevent tv_nsec rollover if whole-second values
     * are packed in it.
     */
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    ts1.tv_sec += ts2.tv_sec;
    ts1.tv_nsec += ts2.tv_nsec;

    return timespec_normalise(ts1);
}

/** \fn struct timespec timespec_sub(struct timespec ts1, struct timespec ts2)
 *  \brief Returns the result of subtracting ts2 from ts1.
 */
struct timespec timespec_sub(struct timespec ts1, struct timespec ts2)
{
    /* Normalise inputs to prevent tv_nsec rollover if whole-second values
     * are packed in it.
     */
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    ts1.tv_sec -= ts2.tv_sec;
    ts1.tv_nsec -= ts2.tv_nsec;

    return timespec_normalise(ts1);
}

/** \fn struct timespec timespec_mod(struct timespec ts1, struct timespec ts2)
 *  \brief Returns the remainder left over after dividing ts1 by ts2 (ts1%ts2).
 */
struct timespec timespec_mod(struct timespec ts1, struct timespec ts2)
{
    int i = 0;
    bool neg1 = false;
    bool neg2 = false;

    /* Normalise inputs to prevent tv_nsec rollover if whole-second values
     * are packed in it.
     */
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    /* If ts2 is zero, just return ts1
     */
    if (ts2.tv_sec == 0 && ts2.tv_nsec == 0)
    {
        return ts1;
    }

    /* If inputs are negative, flip and record sign
     */
    if (ts1.tv_sec < 0 || ts1.tv_nsec < 0)
    {
        neg1 = true;
        ts1.tv_sec = -ts1.tv_sec;
        ts1.tv_nsec = -ts1.tv_nsec;
    }

    if (ts2.tv_sec < 0 || ts2.tv_nsec < 0)
    {
        neg2 = true;
        ts2.tv_sec = -ts2.tv_sec;
        ts2.tv_nsec = -ts2.tv_nsec;
    }

    /* Shift ts2 until it is larger than ts1 or is about to overflow
     */
    while ((ts2.tv_sec < (LONG_MAX >> 1)) && timespec_ge(ts1, ts2))
    {
        i++;
        ts2.tv_nsec <<= 1;
        ts2.tv_sec <<= 1;
        if (ts2.tv_nsec > NSEC_PER_SEC)
        {
            ts2.tv_nsec -= NSEC_PER_SEC;
            ts2.tv_sec++;
        }
    }

    /* Division by repeated subtraction
     */
    while (i >= 0)
    {
        if (timespec_ge(ts1, ts2))
        {
            ts1 = timespec_sub(ts1, ts2);
        }

        if (i == 0)
        {
            break;
        }

        i--;
        if (ts2.tv_sec & 1)
        {
            ts2.tv_nsec += NSEC_PER_SEC;
        }
        ts2.tv_nsec >>= 1;
        ts2.tv_sec >>= 1;
    }

    /* If signs differ and result is nonzero, subtract once more to cross zero
     */
    if (neg1 ^ neg2 && (ts1.tv_sec != 0 || ts1.tv_nsec != 0))
    {
        ts1 = timespec_sub(ts1, ts2);
    }

    /* Restore sign
     */
    if (neg1)
    {
        ts1.tv_sec = -ts1.tv_sec;
        ts1.tv_nsec = -ts1.tv_nsec;
    }

    return ts1;
}

/** \fn bool timespec_eq(struct timespec ts1, struct timespec ts2)
 *  \brief Returns true if the two timespec structures are equal.
 */
bool timespec_eq(struct timespec ts1, struct timespec ts2)
{
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    return (ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec == ts2.tv_nsec);
}

/** \fn bool timespec_gt(struct timespec ts1, struct timespec ts2)
 *  \brief Returns true if ts1 is greater than ts2.
 */
bool timespec_gt(struct timespec ts1, struct timespec ts2)
{
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    return (ts1.tv_sec > ts2.tv_sec || (ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec > ts2.tv_nsec));
}

/** \fn bool timespec_ge(struct timespec ts1, struct timespec ts2)
 *  \brief Returns true if ts1 is greater than or equal to ts2.
 */
bool timespec_ge(struct timespec ts1, struct timespec ts2)
{
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    return (ts1.tv_sec > ts2.tv_sec || (ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec >= ts2.tv_nsec));
}

/** \fn bool timespec_lt(struct timespec ts1, struct timespec ts2)
 *  \brief Returns true if ts1 is less than ts2.
 */
bool timespec_lt(struct timespec ts1, struct timespec ts2)
{
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    return (ts1.tv_sec < ts2.tv_sec || (ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec < ts2.tv_nsec));
}

/** \fn bool timespec_le(struct timespec ts1, struct timespec ts2)
 *  \brief Returns true if ts1 is less than or equal to ts2.
 */
bool timespec_le(struct timespec ts1, struct timespec ts2)
{
    ts1 = timespec_normalise(ts1);
    ts2 = timespec_normalise(ts2);

    return (ts1.tv_sec < ts2.tv_sec || (ts1.tv_sec == ts2.tv_sec && ts1.tv_nsec <= ts2.tv_nsec));
}

struct timespec timespec_normalise(struct timespec ts)
{
    while (ts.tv_nsec >= NSEC_PER_SEC)
    {
        ++(ts.tv_sec);
        ts.tv_nsec -= NSEC_PER_SEC;
    }

    while (ts.tv_nsec <= -NSEC_PER_SEC)
    {
        --(ts.tv_sec);
        ts.tv_nsec += NSEC_PER_SEC;
    }

    if (ts.tv_nsec < 0)
    {
        /* Negative nanoseconds isn't valid according to POSIX.
         * Decrement tv_sec and roll tv_nsec over.
         */

        --(ts.tv_sec);
        ts.tv_nsec = (NSEC_PER_SEC + ts.tv_nsec);
    }

    return ts;
}

uint64_t timespec2ms(struct timespec ts) { return ts.tv_sec * 1000 + ts.tv_nsec / 1000000; }
uint64_t timespec2us(struct timespec ts) { return ts.tv_sec * 1000000 + ts.tv_nsec / 1000; }