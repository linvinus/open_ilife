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
 * @file    templates/continuous_timer_lld.c
 * @brief   CONTINUOUS_TIMER Driver subsystem low level driver source template.
 *
 * @addtogroup CONTINUOUS_TIMER
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "rhal.h"

#if RHAL_USE_CONTINUOUS_TIMER || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#if STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7 || defined(__DOXYGEN__)
static void continuous_timer_callback(GPTDriver *gptp);
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   CONTINUOUS_TIMER1 driver identifier.
 */
CONTINUOUS_TIMERDriver CONTINUOUS_TIMERD;

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/
#if STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7 || defined(__DOXYGEN__)
#define _TIMER_CLOCK_ (84000000/2)
#define _SECONDS_PER_TICK_ ((double)1.0/(double)_TIMER_CLOCK_)
#define _TICKS_PER_ms_ ((double)0.001/(double)_SECONDS_PER_TICK_)
/*84000000/2=42000000 - time clock
 * 1/42000000 = 0,000000023s
 * 0.001s/0,000000023s = 42000 - timer overflow every 1mS */
static GPTConfig gpt7cfg =
{
    _TIMER_CLOCK_,                    /*42Mhz timer clock.*/
    continuous_timer_callback,     /* Timer callback.*/
    0,
    0
};
#endif

#if STM32_CONTINUOUS_TIMER_USE_SYSTICK_TIMER || defined(__DOXYGEN__)
#define _TIMER_CLOCK_ (STM32_HCLK)
#define _SECONDS_PER_TICK_ ((double)1.0/(double)_TIMER_CLOCK_)
#define _TICKS_PER_ms_ (uint32_t)((double)0.001/(double)_SECONDS_PER_TICK_)

#endif


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
#if STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7 || defined(__DOXYGEN__)
static void continuous_timer_callback(GPTDriver *gptp)
{
(void) gptp;
 CONTINUOUS_TIMERD.millis++;
}
#endif

/**
 * @brief   System Timer vector.
 * @details This interrupt is used for system tick in periodic mode.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SysTick_Handler) {

  OSAL_IRQ_PROLOGUE();

  osalSysLockFromISR();
  //~ osalOsTimerHandlerI();
  CONTINUOUS_TIMERD.millis++;
  osalSysUnlockFromISR();

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CONTINUOUS_TIMER driver initialization.
 *
 * @notapi
 */
void continuous_timer_lld_init(void) {

#if STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7
  /* Driver initialization.*/
  continuous_timerObjectInit(&CONTINUOUS_TIMERD);
//#endif /* STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7 */
		gptStart(&GPTD7, &gpt7cfg);
		gptStartContinuous(&GPTD7, _TICKS_PER_ms_); // dT = 36000000Г·36000 = 1,000Hz - 1mS

#endif /* STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7 */

#if STM32_CONTINUOUS_TIMER_USE_SYSTICK_TIMER || defined(__DOXYGEN__)

 SysTick->LOAD = _TICKS_PER_ms_ - 1;//down counter, irq on 1->0
 SysTick->VAL  = _TICKS_PER_ms_ - 1;
 SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                 SysTick_CTRL_TICKINT_Msk   |
                 SysTick_CTRL_ENABLE_Msk;
  /* IRQ enabled.*/
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK, STM32_ST_IRQ_PRIORITY);
#endif

}


inline uint32_t continuous_timer_lld_get_nanos(){
	//~ chSysLockFromISR();//lock from IRQ
#if STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7
	uint16_t nanos = GPTD7.tim->CNT;
#endif

#if STM32_CONTINUOUS_TIMER_USE_SYSTICK_TIMER || defined(__DOXYGEN__)
  uint32_t nanos = SysTick->VAL; //72000 max
  nanos = _TICKS_PER_ms_ - nanos;//down counter
  nanos *= (uint32_t)( (uint32_t)(_SECONDS_PER_TICK_*(1000000000ULL*1000ULL)) + 1UL);//round nanos 13889*72000 < ULONG_MAX
  nanos /=1000UL;//uint32_t round nanos,1000008 max
  nanos -= (uint32_t)nanos/125000UL;//more accurate,1000000 max
#endif
	return nanos;//uint32_t 
}

//~ void continuous_timer_lld_sleep_sleepmicros(uint16_t sleepmicros){
  //~ uint32_t T1 = continuous_timer_lld_get_micros()+sleepmicros;
  //~ while(continuous_timer_lld_get_micros() < T1);
//~ }

void continuous_timer_lld_sleep_nanos(uint16_t sleepnano){
#if STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7

  if(sleepnano>(36000-2)) return; //error!

	uint16_t T1 = GPTD7.tim->CNT;
	uint16_t T2 = T1 + sleepnano;
  while( GPTD7.tim->CNT > T1 && (GPTD7.tim->CNT + T1)< T2 ) ;
#endif
}

#endif //RHAL_USE_CONTINUOUS_TIMER || defined(__DOXYGEN__)
/** @} */
