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
 * @file    templates/continuous_timer_lld.h
 * @brief   CONTINUOUS_TIMER Driver subsystem low level driver header template.
 *
 * @addtogroup CONTINUOUS_TIMER
 * @{
 */

#ifndef _CONTINUOUS_TIMER_LLD_H_
#define _CONTINUOUS_TIMER_LLD_H_

#if RHAL_USE_CONTINUOUS_TIMER || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   CONTINUOUS_TIMER driver enable switch.
 * @details If set to @p TRUE the support for CONTINUOUS_TIMER1 is included.
 */
#if !defined(STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7) || defined(__DOXYGEN__)
#define STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7             FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an CONTINUOUS_TIMER driver.
 */
typedef struct CONTINUOUS_TIMERDriver CONTINUOUS_TIMERDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {

} CONTINUOUS_TIMERConfig;

/**
 * @brief   Structure representing an CONTINUOUS_TIMER driver.
 */
struct CONTINUOUS_TIMERDriver {
  /**
   * @brief Driver state.
   */
  continuous_timerstate_t                state;

  uint32_t millis;  //max 4,294,967,295

  /**
   * @brief Current configuration data.
   */
  const CONTINUOUS_TIMERConfig           *config;
  /* End of the mandatory fields.*/
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

//#if STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7 && !defined(__DOXYGEN__)
//extern CONTINUOUS_TIMERDriver CONTINUOUS_TIMERD7;
//#endif

extern CONTINUOUS_TIMERDriver CONTINUOUS_TIMERD;

#ifdef __cplusplus
extern "C" {
#endif
  void continuous_timer_lld_init(void);
  /*void continuous_timer_lld_start(CONTINUOUS_TIMERDriver *continuous_timerp);
  void continuous_timer_lld_stop(CONTINUOUS_TIMERDriver *continuous_timerp);*/
  inline uint32_t continuous_timer_lld_get_nanos(void) __attribute__((always_inline));
  //~ void continuous_timer_lld_sleep_nanos(uint16_t sleepnano);
  //~ void continuous_timer_lld_sleep_sleepmicros(uint16_t sleepmicros);
#ifdef __cplusplus
}
#endif

#endif /* RHAL_USE_CONTINUOUS_TIMER */

#endif /* _CONTINUOUS_TIMER_LLD_H_ */

/** @} */
