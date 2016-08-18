/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011 Giovanni Di Sirio.

	ORFA2 - Copyright (C) 2012 Vladimir Ermakov.

    This file is part of ORFA2 (dereived from ChibiOS/RT hal.h).

    ORFA2 is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ORFA2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    rhal.h
 * @brief   ORFA HAL subsystem header.
 *
 * @addtogroup RHAL
 * @{
 */

#ifndef _RHAL_H_
#define _RHAL_H_

#define RHAL_USE_CONTINUOUS_TIMER TRUE
#define STM32_CONTINUOUS_TIMER_USE_CONTINUOUS_TIMER7 FALSE
#define STM32_CONTINUOUS_TIMER_USE_SYSTICK_TIMER TRUE

#include "hal.h"

#include "continuous_timer.h"

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void rhalInit(void);
#ifdef __cplusplus
}
#endif

#endif /* _RHAL_H_ */

/** @} */
