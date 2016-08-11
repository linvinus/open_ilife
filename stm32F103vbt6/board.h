/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the Olimex STM32-P103 proto board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_STM32_P103
#define BOARD_NAME              "ilife STM32F103VBT6"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0U
#define STM32_HSECLK            8000000U

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_MD

/*
 * IO pins assignments.
 */
#define GPIOA_PA00  0
#define GPIOA_PA01  1
#define GPIOA_PA02  2
#define GPIOA_PA03  3
#define GPIOA_PA04  4
#define GPIOA_PA05  5
#define GPIOA_PA06  6
#define GPIOA_PA07  7
#define GPIOA_PA08  8
#define GPIOA_PA09_USART1_TX  9
#define GPIOA_PA10_USART1_RX  10
#define GPIOA_PA11  11
#define GPIOA_PA12  12
#define GPIOA_PA13  13
#define GPIOA_PA14  14
#define GPIOA_PA15  15

#define GPIOB_PB00  0
#define GPIOB_PB01  1
#define GPIOB_PB02  2
#define GPIOB_PB03  3
#define GPIOB_PB04  4
#define GPIOB_PB05  5
#define GPIOB_PB06  6
#define GPIOB_PB07  7
#define GPIOB_PB08  8
#define GPIOB_PB09  9
#define GPIOB_PB10  10
#define GPIOB_PB11  11
#define GPIOB_PB12  12
#define GPIOB_PB13  13
#define GPIOB_PB14  14
#define GPIOB_PB15  15


#define GPIOC_PC00  0
#define GPIOC_PC01  1
#define GPIOC_PC02  2
#define GPIOC_PC03  3
#define GPIOC_PC04  4
#define GPIOC_PC05  5
#define GPIOC_PC06  6
#define GPIOC_PC07  7
#define GPIOC_PC08  8
#define GPIOC_PC09  9
#define GPIOC_PC10  10
#define GPIOC_PC11  11
#define GPIOC_PC12  12
#define GPIOC_PC13  13
#define GPIOC_PC14  14
#define GPIOC_PC15  15


#define GPIOD_PD00  0
#define GPIOD_PD01  1
#define GPIOD_PD02  2
#define GPIOD_PD03  3
#define GPIOD_PD04  4
#define GPIOD_PD05  5
#define GPIOD_PD06  6
#define GPIOD_PD07  7
#define GPIOD_PD08  8
#define GPIOD_PD09  9
#define GPIOD_PD10  10
#define GPIOD_PD11  11
#define GPIOD_PD12  12
#define GPIOD_PD13  13
#define GPIOD_PD14  14
#define GPIOD_PD15  15

#define GPIOE_PE00  0
#define GPIOE_PE01  1
#define GPIOE_PE02  2
#define GPIOE_PE03  3
#define GPIOE_PE04  4
#define GPIOE_PE05  5
#define GPIOE_PE06  6
#define GPIOE_PE07  7
#define GPIOE_PE08  8
#define GPIOE_PE09  9
#define GPIOE_PE10  10
#define GPIOE_PE11  11
#define GPIOE_PE12  12
#define GPIOE_PE13  13
#define GPIOE_PE14  14
#define GPIOE_PE15  15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA0  - Normal input      (BUTTON).
 * PA2  - Alternate output  (USART2 TX).
 * PA3  - Normal input      (USART2 RX).
 * PA11 - Normal input      (USB DM).
 * PA12 - Normal input      (USB DP).
 */
#define VAL_GPIOACRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888884B8      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB13 - Alternate output  (MMC SPI2 SCK).
 * PB14 - Normal input      (MMC SPI2 MISO).
 * PB15 - Alternate output  (MMC SPI2 MOSI).
 */
#define VAL_GPIOBCRL            0x88888888      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC4  - Normal input because there is an external resistor.
 * PC6  - Normal input because there is an external resistor.
 * PC7  - Normal input because there is an external resistor.
 * PC10 - Push Pull output (CAN CNTRL).
 * PC11 - Push Pull output (USB DISC).
 * PC12 - Push Pull output (LED).
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88888888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
//#define usb_lld_connect_bus(usbp) palClearPad(GPIOC, GPIOC_USB_DISC)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
//#define usb_lld_disconnect_bus(usbp) palSetPad(GPIOC, GPIOC_USB_DISC)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
