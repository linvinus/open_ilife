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

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"

int FPORT=-1,FPAD=-1;

#define BLDCM_FREQ    72000000 /*CK_CNT=72000000 0.0000156250 s*/
#define BLDCM_PERIOD  3500


static PWMConfig pwm_bldc_cfg = {
  BLDCM_FREQ,
  BLDCM_PERIOD,
  /*pwm_timer_uev*/NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH , NULL},
    {PWM_OUTPUT_ACTIVE_HIGH , NULL},
    {PWM_OUTPUT_ACTIVE_HIGH , NULL},
    {PWM_OUTPUT_ACTIVE_HIGH , NULL}/*NULL*/
  },
  /* HW */
  /*cr2*/ 0,
  /*bdtr*/0,
  0
};

/*
 * Blinker thread.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (true) {
    //~ palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(500);
    //~ palClearPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(500);
    //~ chprintf(&SD1, "ilife!\r\n");
    //~ chprintf(&SD1, "PORT=%d PAD=%d\r\n",FPORT,FPAD);
    //~ sd_flush(&SD1);
  }
}

void sd_flush(SerialDriver *sdp){
      while(1){
        chSysLock();
        if(chOQIsEmptyI(&(sdp->oqueue))) break;
        chSysUnlock();
        chThdSleepMilliseconds(1);
      }
      chSysUnlock();
}

int port_state[5]={0,0,0,0,0};
GPIO_TypeDef *GPIOS[5]={GPIOA,GPIOB,GPIOC,GPIOD,GPIOE};
int mask[5]={
            1<<GPIOA_PA09_USART1_TX | 1<<GPIOA_PA10_USART1_RX | 1<<GPIOA_PA00_TOUCH_BUTTON | 1<<GPIOA_PA13_SWDIO | 1<<GPIOA_PA14_SWCLK | 1<<GPIOA_PA08_SIDE_BRUSH_ENABLE | 1<<GPIOA_PA11_BEEPER | 1<<GPIOA_PA03_GROUND_SENSOR_RIGHT,
            1<<GPIOB_PB07_MOT_L_PHASE | 1<<GPIOB_PB13_SPI2_SCK | 1<<GPIOB_PB05_CONTACT_BUMPER_L | 1<<GPIOB_PB11_IC_FRONTRIGHT | 1<<GPIOB_PB14_TOURBINE_ENABLE | 1<<GPIOB_PB10_GROUND_SENSORS_TX,
            1<<GPIOC_PC06_MOT_R_ENABLE | 1<<GPIOC_PC08_MOT_L_ENABLE | 1<<GPIOC_PC09_MAIN_BRUSH_ENABLE | 1<<GPIOC_PC02_GROUND_SENSOR_FRONTLEFT | 1<<GPIOC_PC04_GROUND_SENSOR_FRONTRIGHT | 1<<GPIOC_PC01_GROUND_SENSOR_LEFT,
            1<<GPIOD_PD03_ENC_L | 1<<GPIOD_PD10_MOT_R_GROUND | 1<<GPIOD_PD04_IC_REAR | 1<<GPIOD_PD13_IC_LEFT | 1<<GPIOD_PD11_TOUCH_BUTTON_COLOR_1 ,
            1<<GPIOE_PE05_SLEEP | 1<<GPIOE_PE13_MOT_R_PHASE | 1<<GPIOE_PE08_ENC_R | 1<<GPIOE_PE12_CONTACT_BUMPER_R | 1<<GPIOE_PE03_MOT_L_GROUND | 1<<GPIOE_PE06_IC_RIGHT | 1<<GPIOE_PE10_IC_FRONTLEFT | 1<<GPIOE_PE00_BUG_LEFT_MAGNET | 1<<GPIOE_PE04_BUG_RIGHT_MAGNET | 1<<GPIOE_PE11_TOUCH_BUTTON_COLOR_2
            };


void test_pad(){
  int i,j,k;
  FPORT=-1;
  FPAD=-1;

  palSetPadMode(GPIOB, GPIOB_PB13_SPI2_SCK, PAL_MODE_INPUT_PULLDOWN);//TEST PAD
  chprintf(&SD1, "PREPARE TEST\r\n");
  //~ chThdSleepMilliseconds(10000);
  chprintf(&SD1, "TESTING...\r\n");
  for(i=0;i<5;i++){
    for(j=0;j<16;j++){

      if(GPIOS[i]==GPIOB && j== GPIOB_PB13_SPI2_SCK) continue;//test pad
      //~ if(GPIOS[i]==GPIOD && j== GPIOD_PD00_XTAL) continue;
      //~ if(GPIOS[i]==GPIOD && j== GPIOD_PD01_XTAL) continue;
      //~ if(GPIOS[i]==GPIOA && j== GPIOA_PA09_USART1_TX) continue;
      //~ if(GPIOS[i]==GPIOA && j== GPIOA_PA10_USART1_RX) continue;

      //~ if(GPIOS[i]==GPIOB && j== GPIOB_PB07_MOT_L_PHASE) continue;
      //~ if(GPIOS[i]==GPIOC && j== GPIOC_PC08_MOT_L_ENABLE) continue;
      //~ if(GPIOS[i]==GPIOE && j== GPIOE_PE13_MOT_R_PHASE) continue;
      //~ if(GPIOS[i]==GPIOC && j== GPIOC_PC06_MOT_R_ENABLE) continue;
      //~ if(GPIOS[i]==GPIOC && j== GPIOC_PC09_MAIN_BRUSH_ENABLE) continue;
      if(mask[i] & (1<<j)) continue;

      //~ if(GPIOS[i]==GPIOA && j== 10) continue;//beeper
      chprintf(&SD1, "[%d,%d]\r\n",i,j);
      sd_flush(&SD1);

      palSetPadMode(GPIOS[i], j, PAL_MODE_OUTPUT_PUSHPULL);
      for(k=0;k<10;k++){
        palSetPad(GPIOS[i], j);
        chThdSleepMilliseconds(10);
        palClearPad(GPIOS[i], j);
        chThdSleepMilliseconds(10);
      }

      if( palReadPad(GPIOB, GPIOB_PB13_SPI2_SCK) > 0){
        FPORT=i;
        FPAD=j;
        chprintf(&SD1, "PORT=%d PAD=%d\r\n",FPORT,FPAD);
        //~ goto done;
        //~ break;
       }
      palClearPad(GPIOS[i], j);
      palSetPadMode(GPIOS[i], j, PAL_MODE_INPUT);
      sdGet(&SD1);//wait for key
    }
  }
  chprintf(&SD1, "TEST DONE\r\n");
}

void compare_pads(){
  int i,j;
  //~ chprintf(&SD1, "compare_pads\r\n");
  for(i=0;i<5;i++){
    int tmp = (palReadPort(GPIOS[i]) &~ mask[i]) & 0xFFFF ;
    if(tmp != port_state[i]){
      int tmp2 = tmp ^ port_state[i];
      chprintf(&SD1, "<%c> 0x%04x 0x%04x!=0x%04x ",'A'+i,tmp2,tmp,port_state[i]);
      for(j=0;j<15;j++){
        if( (tmp2 & (1<<j)) !=0 )
          chprintf(&SD1, "P%c%d ",'A'+i,j);
        }
      chprintf(&SD1, "\r\n");
      sd_flush(&SD1);
      port_state[i] = tmp;
    }
  }

}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1 | AFIO_MAPR_TIM3_REMAP_0;
  //RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;/*!< JTAG-DP Disabled and SW-DP Enabled */


  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9(TX) and PA10(RX) are routed to USART1.
   */
  sdStart(&SD1, NULL);
  //~ palSetPadMode(GPIOA, GPIOA_PA09_USART1_TX, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  //~ palSetPadMode(GPIOA, GPIOA_PA10_USART1_RX, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  chprintf(&SD1, "hello ilife!\r\n");

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);


  //~ palSetPadMode(GPIOE, GPIOE_PE05_SLEEP, PAL_MODE_OUTPUT_PUSHPULL);//SLEEP OK
  palSetPad(GPIOE, GPIOE_PE05_SLEEP);//wakeup
  chThdSleepMilliseconds(1);//wait DRV8801


  //~ palSetPad(GPIOB, GPIOB_PB07_MOT_L_PHASE);
  //~ palSetPad(GPIOC, GPIOC_PC08_MOT_L_ENABLE);
  //~ chThdSleepMilliseconds(500);
  //~ palClearPad(GPIOB, GPIOB_PB07_MOT_L_PHASE);
  //~ chThdSleepMilliseconds(500);
  //~ palClearPad(GPIOC, GPIOC_PC08_MOT_L_ENABLE);

  //~ palSetPad(GPIOE, GPIOE_PE13_MOT_R_PHASE);
  //~ palSetPad(GPIOC, GPIOC_PC06_MOT_R_ENABLE);
  //~ chThdSleepMilliseconds(500);
  //~ palClearPad(GPIOE, GPIOE_PE13_MOT_R_PHASE);
  //~ chThdSleepMilliseconds(500);
  //~ palClearPad(GPIOC, GPIOC_PC06_MOT_R_ENABLE);

  palSetPadMode(GPIOB, GPIOB_PB10_GROUND_SENSORS_TX, PAL_MODE_OUTPUT_PUSHPULL);

  //~ palSetPadMode(GPIOC, GPIOC_PC07, PAL_MODE_OUTPUT_PUSHPULL);//SLEEP OK
  //~ palSetPad(GPIOC, GPIOC_PC07);

  pwmStart(&PWMD3, &pwm_bldc_cfg);

  pwmEnableChannel(&PWMD3, 0,3500/4);//tim3-ch1
  pwmEnableChannel(&PWMD3, 2,3500/4);//tim3-ch3

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched.
   */
  while (true) {
    compare_pads();
    chThdSleepMilliseconds(100);
    palTogglePort(GPIOB, 1<<GPIOB_PB10_GROUND_SENSORS_TX);

    //~ if (sdGetTimeout(&SD1,MS2ST(100))>0){
      //~ test_pad();
    //~ }//else{
      //~ chThdSleepMilliseconds(500);
    //~ }
  }
}
