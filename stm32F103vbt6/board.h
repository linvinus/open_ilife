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
#define GPIOA_PA00_TOUCH_BUTTON  0      /*ADC12_IN0 (digital)*/
#define GPIOA_PA01_CHARGER_VOLTAGE  1   /*ADC12_IN1 charger_voltage*/
#define GPIOA_PA02_BATT_VOLTAGE  2      /*ADC12_IN2 batt_voltage*/
#define GPIOA_PA03_GROUND_SENSOR_RIGHT  3 /*ADC12_IN3*/
#define GPIOA_PA04_TOURBINE_CURRENT  4  /*ADC12_IN4*/
#define GPIOA_PA05_IRBUMPER_CENTER  5   /*ADC12_IN5, 3 phototransistors in parallel*/
#define GPIOA_PA06_BRUSH_CURRENT  6     /*ADC12_IN6*/
#define GPIOA_PA07_BATT_CURRENT  7      /*ADC12_IN7 current sensor*/
#define GPIOA_PA08_SIDE_BRUSH_ENABLE  8 /*TIM1_CH1*/
#define GPIOA_PA09_USART1_TX  9
#define GPIOA_PA10_USART1_RX  10
#define GPIOA_PA11_BEEPER  11
#define GPIOA_PA12  12                   /*71 ????*/
#define GPIOA_PA13_SWDIO  13
#define GPIOA_PA14_SWCLK  14
#define GPIOA_PA15  15                   /*unused*/

#define GPIOB_PB00_IRBUMPER_LEFT  0      /*ADC12_IN8, 4 phototransistors in parallel*/
#define GPIOB_PB01  1                    /*36 ????*/ /*ADC12_IN9, not used*/
#define GPIOB_PB02  2                    /*37 ????*/
#define GPIOB_PB03  3                    /*89 ????*/
#define GPIOB_PB04  4                    /*90 unused*/
#define GPIOB_PB05_CONTACT_BUMPER_L  5
#define GPIOB_PB06  6                    /*92 unused*/
#define GPIOB_PB07_MOT_L_PHASE  7
#define GPIOB_PB08  8                    /*95 unused*/
#define GPIOB_PB09  9                    /*96 unused*/
#define GPIOB_PB10_GROUND_SENSORS_TX  10
#define GPIOB_PB11_IR_FRONTRIGHT  11
#define GPIOB_PB12  12                   /*33 ????*/
#define GPIOB_PB13_SPI2_SCK  13
#define GPIOB_PB14_TOURBINE_ENABLE  14     /*TIM1_CH2N*/
#define GPIOB_PB15  15                   /*36 ????*/


#define GPIOC_PC00_GROUND_SENSOR_LEFT  0      /*ADC12_IN10*/
#define GPIOC_PC01_IRBUMPER_LEFT_WALL  1      /*ADC12_IN11*/
#define GPIOC_PC02_GROUND_SENSOR_FRONTLEFT  2 /*ADC12_IN12*/
#define GPIOC_PC03_IRBUMPER_RIGHT  3          /*ADC12_IN13, 4 phototransistors in parallel*/
#define GPIOC_PC04_GROUND_SENSOR_FRONTRIGHT  4 /*ADC12_IN14*/
#define GPIOC_PC05  5                         /*34 ????*/ /*ADC12_IN15, not used*/
#define GPIOC_PC06_MOT_R_ENABLE  6            /*TIM3_CH1*/
#define GPIOC_PC07  7                         /*64 ???? charger_enable*/ /*TIM3_CH2 должна быть боковая щётка но нет*/
#define GPIOC_PC08_MOT_L_ENABLE  8            /*TIM3_CH3*/
#define GPIOC_PC09_MAIN_BRUSH_ENABLE  9       /*TIM3_CH4*/
#define GPIOC_PC10  10                        /*51 ????*/
#define GPIOC_PC11  11                        /*52 ????*/
#define GPIOC_PC12  12                        /*53 ????*/
#define GPIOC_PC13  13                        /*7 unused*/
#define GPIOC_PC14  14                        /*8 ????*/
#define GPIOC_PC15  15                        /*9 ????*/


#define GPIOD_PD00  0                         /*81 ????*/
#define GPIOD_PD01  1                         /*82 unused*/
#define GPIOD_PD02_ENC_CASTER_WHEEL  2
#define GPIOD_PD03_ENC_L  3
#define GPIOD_PD04_IR_REAR  4
#define GPIOD_PD05  5                         /*86 unused*/
#define GPIOD_PD06  6                         /*87 unused*/
#define GPIOD_PD07  7                         /*88 unused*/
#define GPIOD_PD08  8                         /*55 ???? 5v on sleep_off*/
#define GPIOD_PD09  9                         /*56 unused*/
#define GPIOD_PD10_MOT_R_GROUND  10
#define GPIOD_PD11_TOUCH_BUTTON_COLOR_1  11
#define GPIOD_PD12  12                        /*59 ????*/
#define GPIOD_PD13_IR_LEFT  13
#define GPIOD_PD14  14                        /*61 ???? 5v on sleep_off*/
#define GPIOD_PD15  15                        /*62 unused*/

#define GPIOE_PE00_BUG_LEFT_MAGNET  0
#define GPIOE_PE01  1                         /*98 unused*/
#define GPIOE_PE02  2                         /*01 unused*/
#define GPIOE_PE03_MOT_L_GROUND  3
#define GPIOE_PE04_BUG_RIGHT_MAGNET  4
#define GPIOE_PE05_SLEEP  5
#define GPIOE_PE06_IR_RIGHT  6
#define GPIOE_PE07  7                         /*38 unused*/
#define GPIOE_PE08_ENC_R  8
#define GPIOE_PE09  9                         /*40 unused*/
#define GPIOE_PE10_IR_FRONTLEFT  10
#define GPIOE_PE11_TOUCH_BUTTON_COLOR_2  11
#define GPIOE_PE12_CONTACT_BUMPER_R  12
#define GPIOE_PE13_MOT_R_PHASE  13
#define GPIOE_PE14  14                        /*45 ????*/
#define GPIOE_PE15  15                        /*46 unused*/

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
#define VAL_GPIOACRL            0x44444444      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x444444B4      /* PA15...PA8 */
#define VAL_GPIOAODR            0x00000000

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB13 - Alternate output  (MMC SPI2 SCK).
 * PB14 - Normal input      (MMC SPI2 MISO).
 * PB15 - Alternate output  (MMC SPI2 MOSI).
 */
#define VAL_GPIOBCRL            0x24444444      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x44444444      /* PB15...PB8 */
#define VAL_GPIOBODR            0x00000000

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
#define VAL_GPIOCCRL            0x4A444444      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x444444AA      /* PC15...PC8 */
#define VAL_GPIOCODR            0x00000000

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#define VAL_GPIODCRL            0x44444444      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x44444444      /* PD15...PD8 */
#define VAL_GPIODODR            0x00000000

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x44244444      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x44244444      /* PE15...PE8 */
#define VAL_GPIOEODR            0x00000000

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
