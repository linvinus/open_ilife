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
 * @file    continuous_timer.h
 * @brief   continuous_timer Driver macros and structures.
 *
 * @addtogroup continuous_timer
 * @{
 */

#ifndef _CONTINUOUS_TIMER_H_
#define _CONTINUOUS_TIMER_H_

#if RHAL_USE_CONTINUOUS_TIMER || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  CONTINUOUS_TIMER_UNINIT = 0,                   /**< Not initialized.                   */
  CONTINUOUS_TIMER_STOP = 1,                     /**< Stopped.                           */
  CONTINUOUS_TIMER_READY = 2,                    /**< Ready.                             */
} continuous_timerstate_t;

/**
 * @brief   Type of a structure representing a continuous_timer driver.
 */
typedef struct continuous_timerDriver continuous_timerDriver;

#include "continuous_timer_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

typedef struct {
	uint64_t prev_time; /*last position before last call of encoderGetDelta*/
	int64_t last_delta; /*last delta after last call of encoderGetDelta*/
} continous_timer_state;

#ifdef __cplusplus
extern "C" {
#endif
  void continuous_timerInit(void);
  void continuous_timerObjectInit(CONTINUOUS_TIMERDriver *continuous_timerp);

  inline uint32_t continuous_timer_get_millis(void) __attribute__((always_inline));

  inline uint64_t continuous_timer_get_microsI(void) __attribute__((always_inline));
  uint64_t continuous_timer_get_micros(void);

  inline uint64_t continuous_timer_get_nanosI(void) __attribute__((always_inline));
  uint64_t continuous_timer_get_nanos(void);
  
  inline int32_t continuous_timer_get_micros_delta32I(continous_timer_state *state) __attribute__((always_inline));
  int32_t continuous_timer_get_micros_delta32(continous_timer_state *state);
  
  inline int64_t continuous_timer_get_micros_delta64I(continous_timer_state *state) __attribute__((always_inline));
  int64_t continuous_timer_get_micros_delta64(continous_timer_state *state);
  
  inline int32_t continuous_timer_get_millis_deltaI(continous_timer_state *state, bool update) __attribute__((always_inline));
  int32_t continuous_timer_get_millis_delta(continous_timer_state *state, bool update);

  /*void continuous_timerStart(continuous_timerDriver *continuous_timerp, const CONTINUOUS_TIMERConfig *config);
  void continuous_timerStop(continuous_timerDriver *continuous_timerp);*/
#ifdef __cplusplus
}
#endif

#endif /* RHAL_USE_CONTINUOUS_TIMER */

#endif /* _CONTINUOUS_TIMER_H_ */

/** @} */
