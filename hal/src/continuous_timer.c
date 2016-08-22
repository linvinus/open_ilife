/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */


/**
 * @file    continuous_timer.c
 * @brief   CONTINUOUS_TIMER Driver code.
 *
 * @addtogroup CONTINUOUS_TIMER
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "rhal.h"
#include "limits.h"
#include "continuous_timer_lld.h"

#if RHAL_USE_CONTINUOUS_TIMER || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   CONTINUOUS_TIMER Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void continuous_timerInit(void) {

  continuous_timer_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p CONTINUOUS_TIMERDriver structure.
 *
 * @param[out] continuous_timerp     pointer to the @p CONTINUOUS_TIMERDriver object
 *
 * @init
 */
void continuous_timerObjectInit(CONTINUOUS_TIMERDriver *continuous_timerp) {

  continuous_timerp->state  = CONTINUOUS_TIMER_STOP;
  continuous_timerp->config = NULL;
}

/**
 * @brief   Configures and activates the CONTINUOUS_TIMER peripheral.
 *
 * @param[in] continuous_timerp      pointer to the @p CONTINUOUS_TIMERDriver object
 * @param[in] config    pointer to the @p CONTINUOUS_TIMERConfig object
 *
 * @api
 */
/*void continuous_timerStart(CONTINUOUS_TIMERDriver *continuous_timerp, const CONTINUOUS_TIMERConfig *config) {

  chDbgCheck((continuous_timerp != NULL) && (config != NULL), "continuous_timerStart");

  chSysLock();
  chDbgAssert((continuous_timerp->state == CONTINUOUS_TIMER_STOP) || (continuous_timerp->state == CONTINUOUS_TIMER_READY),
              "continuous_timerStart(), #1", "invalid state");
  continuous_timerp->config = config;
  continuous_timer_lld_start(continuous_timerp);
  continuous_timerp->state = CONTINUOUS_TIMER_READY;
  chSysUnlock();
}*/

/**
 * @brief   Deactivates the CONTINUOUS_TIMER peripheral.
 *
 * @param[in] continuous_timerp      pointer to the @p CONTINUOUS_TIMERDriver object
 *
 * @api
 */
/*void continuous_timerStop(CONTINUOUS_TIMERDriver *continuous_timerp) {

  chDbgCheck(continuous_timerp != NULL, "continuous_timerStop");

  chSysLock();
  chDbgAssert((continuous_timerp->state == CONTINUOUS_TIMER_STOP) || (continuous_timerp->state == CONTINUOUS_TIMER_READY),
              "continuous_timerStop(), #1", "invalid state");
  continuous_timer_lld_stop(continuous_timerp);
  continuous_timerp->state = CONTINUOUS_TIMER_STOP;
  chSysUnlock();
}*/


inline uint32_t continuous_timer_get_millis(){
  return CONTINUOUS_TIMERD.millis; //uint32_t; overflow occur every 49days
}

/* overflow will occur every
 * 4294967295/1000 - (1000008/1000) =  4293967ms (~71min)
 * */
inline uint32_t continuous_timer_get_micros32I(void){
  uint32_t n;
  uint32_t m;
 	m = continuous_timer_get_millis(); //uint32_t
  n = continuous_timer_lld_get_nanos();//uint32_t 1000008 max
  m *= 1000UL;//ms ->us
  m += (n + 500UL)/1000UL;//round ns -> us then add
  return m;
}

inline uint64_t continuous_timer_get_micros64I(void){
  uint32_t n;
  uint64_t m;
 	m = continuous_timer_get_millis(); //uint32_t
  n = continuous_timer_lld_get_nanos();//uint32_t
  m *= 1000ULL;//ms ->us
  m += (n + 500ULL)/1000ULL;//round ns -> us
  return m;
}

uint64_t continuous_timer_get_micros(){
  uint64_t m;
  chSysLock();
  m = continuous_timer_get_microsI();
  chSysUnlock();
  return m;
}

inline uint64_t continuous_timer_get_nanosI(){
  uint32_t n;
  uint64_t m;
  m = continuous_timer_get_millis(); //uint32_t
  n = continuous_timer_lld_get_nanos(); //uint32_t
  m *= 1000000ULL;//ms ->ns
  m += n;
return m;
}

uint64_t continuous_timer_get_nanos(){
  uint64_t n;
  chSysLock();
  n = continuous_timer_get_nanosI();
  chSysUnlock();
return n;
}

/*
 * Handle big difference in timer overflow case, (occur every 49days for uint32 counter ms)
 * max dt for us is 1073741824us (1073s ~17min)
 * max dt for ms is 1073741824ms (~298hrs)
 * */
static inline int32_t normalize_dt32(int32_t dt)
{
	//signed int32 max value is 2147483647
    if (dt > ((LONG_MAX - 0) / 2L))
    {
        dt -= (LONG_MAX - 0);
    }

    if (dt < (LONG_MIN - 0) / 2L)
    {
        dt += (LONG_MAX - 0);
    }

    return dt;
}
/*
 * Handle big difference in timer overflow case,
 * max dt for us is 4611686018427387903us (~146235years)
 * max dt for ns is 4611686018427387903ns (~146years)
 * */
static inline int64_t normalize_dt64(int64_t dt)
{
    if (dt > ((LONG_LONG_MAX - 0) / 2LL))
    {
        dt -= (LONG_LONG_MAX - 0);
    }

    if (dt < ((LONG_LONG_MIN - 0) / 2LL))
    {
        dt += (LONG_LONG_MAX - 0);
    }

    return dt;
}
/* continuous_timer_get_micros_delta32I
 * max delta  is 1073741824us (1073s ~17min)
 * */
inline int32_t continuous_timer_get_micros_delta32I(continous_timer_state *state){
  uint32_t u;
  u = continuous_timer_get_micros32I();
  state->last_delta = normalize_dt32( u - state->prev_time);
  state->prev_time = u;
  return state->last_delta;
}

inline int64_t continuous_timer_get_micros_delta64I(continous_timer_state *state){
  uint64_t u;
  u = continuous_timer_get_micros64I();
  state->last_delta = normalize_dt64( u - state->prev_time );
  state->prev_time = u;
  return state->last_delta;
}

int32_t continuous_timer_get_micros_delta32(continous_timer_state *state){
  chSysLock();
    continuous_timer_get_micros_delta32I(state);
  chSysUnlock();
  return state->last_delta;
}

int64_t continuous_timer_get_micros_delta64(continous_timer_state *state){
  chSysLock();
    continuous_timer_get_micros_delta64I(state);
  chSysUnlock();
  return state->last_delta;
}

inline  int32_t continuous_timer_get_millis_deltaI(continous_timer_state *state, bool update){
  uint32_t t;
  t=continuous_timer_get_millis();
  state->last_delta=normalize_dt32(t - state->prev_time);
  if(update)
    state->prev_time = t;
  return state->last_delta;
}

int32_t continuous_timer_get_millis_delta(continous_timer_state *state, bool update){
  chSysLock();
  continuous_timer_get_millis_deltaI(state, update);
  chSysUnlock();
  return state->last_delta;
}

#endif /* RHAL_USE_CONTINUOUS_TIMER */

/** @} */
