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
#include "rhal.h"
#include "test.h"
#include "chprintf.h"

#include "serial_protocol_modules.h"

#define SERIAL (BaseSequentialStream *) &SD1

int FPORT=-1,FPAD=-1;

//~ static
adcsample_t adc1_samples_buf[16]; // results array

//~ static
ADCConversionGroup bldc_adc1_grp_for_tim1 = {
	// set to TRUE if need circular buffer, set FALSE otherwise
    FALSE,
	// number of channels
    (uint16_t)(16),
	// callback function, set to NULL for begin
    NULL,
    NULL,
  /* HW dependent part */
  // Resent fields are stm32 specific. They contain ADC control registers data.
  // Please, refer to ST manual RM0008.pdf to understand what we do
  // CR1 register content, set to zero for begin
    0,
  // CR2 register content, set to zero for begin
    ADC_CR2_TSVREFE,
  // SMRP1 register content, set to zero for begin
    ADC_SMPR1_SMP_AN10(ADC_SAMPLE_41P5)|
    ADC_SMPR1_SMP_AN11(ADC_SAMPLE_41P5)|
    ADC_SMPR1_SMP_AN12(ADC_SAMPLE_41P5)|
    ADC_SMPR1_SMP_AN13(ADC_SAMPLE_41P5)|
    ADC_SMPR1_SMP_AN14(ADC_SAMPLE_41P5)|
    ADC_SMPR1_SMP_AN15(ADC_SAMPLE_41P5),
  // SMRP2 register content, set to zero for begin
    ADC_SMPR2_SMP_AN0(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN1(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN2(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN3(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN4(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN5(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN6(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN7(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN8(ADC_SAMPLE_41P5)|
    ADC_SMPR2_SMP_AN9(ADC_SAMPLE_41P5),
  // SQR1 register content. Set channel sequence length
    ADC_SQR1_NUM_CH(16)|
    ADC_SQR1_SQ13_N(ADC_CHANNEL_IN12)|
    ADC_SQR1_SQ14_N(ADC_CHANNEL_IN13)|
    ADC_SQR1_SQ15_N(ADC_CHANNEL_IN14)|
    ADC_SQR1_SQ16_N(ADC_CHANNEL_IN15),
  // SQR2 register content, set to zero for begin
    ADC_SQR2_SQ7_N(ADC_CHANNEL_IN6)|
    ADC_SQR2_SQ8_N(ADC_CHANNEL_IN7)|
    ADC_SQR2_SQ9_N(ADC_CHANNEL_IN8)|
    ADC_SQR2_SQ10_N(ADC_CHANNEL_IN9)|
    ADC_SQR2_SQ11_N(ADC_CHANNEL_IN10)|
    ADC_SQR2_SQ12_N(ADC_CHANNEL_IN11),
  // SQR3 register content. We must select 6 channels
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)|
    ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)|
    ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2)|
    ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3)|
    ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4)|
    ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5)
};



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
  /*bdtr*/0
};



void sd_flush(SerialDriver *sdp){
      while(1){
        chSysLock();
        if(chOQIsEmptyI(&(sdp->oqueue))) break;
        chSysUnlock();
        //~ chThdSleepMilliseconds(1);
        chThdYield();//as fast as possible
      }
      chSysUnlock();
}

int port_state[5]={0,0,0,0,0};
GPIO_TypeDef *GPIOS[5]={GPIOA,GPIOB,GPIOC,GPIOD,GPIOE};
int mask[5]={
            1<<GPIOA_PA09_USART1_TX | 1<<GPIOA_PA10_USART1_RX | 1<<GPIOA_PA00_TOUCH_BUTTON | 1<<GPIOA_PA13_SWDIO | 1<<GPIOA_PA14_SWCLK | 1<<GPIOA_PA08_SIDE_BRUSH_ENABLE | 1<<GPIOA_PA11_BEEPER | 1<<GPIOA_PA03_GROUND_SENSOR_RIGHT | 1<<GPIOA_PA05_IRBUMPER_CENTER | 1<<GPIOA_PA01_CHARGER_VOLTAGE | 1<<GPIOA_PA02_BATT_VOLTAGE | 1<<GPIOA_PA04_TOURBINE_CURRENT | 1<<GPIOA_PA06_BRUSH_CURRENT | 1<<GPIOA_PA07_BATT_CURRENT,
            1<<GPIOB_PB07_MOT_L_PHASE | 1<<GPIOB_PB13_SPI2_SCK | 1<<GPIOB_PB05_CONTACT_BUMPER_L | 1<<GPIOB_PB11_IR_FRONTRIGHT | 1<<GPIOB_PB14_TOURBINE_ENABLE | 1<<GPIOB_PB10_GROUND_SENSORS_TX | 1<<GPIOB_PB00_IRBUMPER_LEFT ,
            1<<GPIOC_PC06_MOT_R_ENABLE | 1<<GPIOC_PC08_MOT_L_ENABLE | 1<<GPIOC_PC09_MAIN_BRUSH_ENABLE | 1<<GPIOC_PC02_GROUND_SENSOR_FRONTLEFT | 1<<GPIOC_PC04_GROUND_SENSOR_FRONTRIGHT | 1<<GPIOC_PC00_GROUND_SENSOR_LEFT | 1<<GPIOC_PC03_IRBUMPER_RIGHT  | 1<<GPIOC_PC01_IRBUMPER_LEFT_WALL,
            1<<GPIOD_PD03_ENC_L | 1<<GPIOD_PD10_MOT_R_GROUND | 1<<GPIOD_PD04_IR_REAR | 1<<GPIOD_PD13_IR_LEFT | 1<<GPIOD_PD11_TOUCH_BUTTON_COLOR_1 ,
            1<<GPIOE_PE05_SLEEP | 1<<GPIOE_PE13_MOT_R_PHASE | 1<<GPIOE_PE08_ENC_R | 1<<GPIOE_PE12_CONTACT_BUMPER_R | 1<<GPIOE_PE03_MOT_L_GROUND | 1<<GPIOE_PE06_IR_RIGHT | 1<<GPIOE_PE10_IR_FRONTLEFT | 1<<GPIOE_PE00_BUG_LEFT_MAGNET | 1<<GPIOE_PE04_BUG_RIGHT_MAGNET | 1<<GPIOE_PE11_TOUCH_BUTTON_COLOR_2
            };


void test_pad(void){
  int i,j,k;
  FPORT=-1;
  FPAD=-1;

  palSetPadMode(GPIOB, GPIOB_PB13_SPI2_SCK, PAL_MODE_INPUT_PULLDOWN);//TEST PAD
  chprintf(SERIAL, "PREPARE TEST\r\n");
  //~ chThdSleepMilliseconds(10000);
  chprintf(SERIAL, "TESTING...\r\n");
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
      chprintf(SERIAL, "[%d,%d]\r\n",i,j);
      sd_flush(&SD1);

      palSetPadMode(GPIOS[i], j, PAL_MODE_OUTPUT_PUSHPULL);
      for(k=0;k<1;k++){
        palSetPad(GPIOS[i], j);
        chThdSleepMilliseconds(1000);
        palClearPad(GPIOS[i], j);
        chThdSleepMilliseconds(10);
      }

      if( palReadPad(GPIOB, GPIOB_PB13_SPI2_SCK) > 0){
        FPORT=i;
        FPAD=j;
        chprintf(SERIAL, "PIN=%c%d\r\n",'A'+FPORT,FPAD);
        //~ goto done;
        //~ break;
       }
      palClearPad(GPIOS[i], j);
      palSetPadMode(GPIOS[i], j, PAL_MODE_INPUT);
      sdGet(&SD1);//wait for key
    }
  }
  chprintf(SERIAL, "TEST DONE\r\n");
}

void compare_pads(void){
  int i,j;
  //~ chprintf(SERIAL, "compare_pads\r\n");
  for(i=0;i<5;i++){
    int tmp = (palReadPort(GPIOS[i]) &~ mask[i]) & 0xFFFF ;
    if(tmp != port_state[i]){
      int tmp2 = tmp ^ port_state[i];
      chprintf(SERIAL, "<%c> 0x%04x 0x%04x!=0x%04x ",'A'+i,tmp2,tmp,port_state[i]);
      for(j=0;j<15;j++){
        if( (tmp2 & (1<<j)) !=0 )
          chprintf(SERIAL, "P%c%d=%d ",'A'+i,j,((tmp>>j) & 1) );
        }
      chprintf(SERIAL, "\r\n");
      sd_flush(&SD1);
      port_state[i] = tmp;
    }
  }

}

/*
 * Blinker thread.
 */
static THD_WORKING_AREA(waThread1, 256);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  continous_timer_state timestate;
  chRegSetThreadName("blinker");
  continuous_timer_get_micros_delta32(&timestate);
  CONTINUOUS_TIMERD.millis=4294967294UL - 1000UL*5UL;
  uint32_t test=0;
  while (true) {
    //~ palSetPad(GPIOC, GPIOC_LED);
    //~ chThdSleepMilliseconds(500);
    //~ palClearPad(GPIOC, GPIOC_LED);
    //~ chThdSleepMilliseconds(500);
    //~ chprintf(SERIAL, "ilife!\r\n");
    //~ chprintf(SERIAL, "PORT=%d PAD=%d\r\n",FPORT,FPAD);
    //~ sd_flush(&SD1);

    uint32_t i;
    //blink
    //palClearPad(GPIOB, GPIOB_PB10_GROUND_SENSORS_TX);
    //chThdSleepMilliseconds(1);
    //palSetPad(GPIOB, GPIOB_PB10_GROUND_SENSORS_TX);

    adcStartConversion(&ADCD1,
                       &bldc_adc1_grp_for_tim1,
                       adc1_samples_buf,
                       1);
    chprintf(SERIAL, "%d ",continuous_timer_get_micros_delta32(&timestate));

    for(i=0; i < test; i++)
      adc1_samples_buf[15]+=adc1_samples_buf[15]/ i;
    test+=1000;

    for(i=0;i<16;i++){
      if (i==10 || i==11 || i==12 || i==14 || i==1 || i==2 || i==3 || i==13 || i==5 || i==8 || i==0 || i==6) continue;
      chprintf(SERIAL, "%02d=%04d ",i,adc1_samples_buf[i]);
    }
    chprintf(SERIAL, " %lu\r\n",CONTINUOUS_TIMERD.millis);
    compare_pads();
    //~ chThdSleepMilliseconds(600);
    sd_flush(&SD1);

  }
}

/*
 * Application entry point.
 */
int main(void) {
  int motor_en=0,brush_en=0;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  rhalInit();//ORFA

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
  chprintf(SERIAL, "hello ilife!\r\n");



  palSetPadMode(GPIOE, GPIOE_PE05_SLEEP, PAL_MODE_OUTPUT_PUSHPULL);//SLEEP OK
  //~ palSetPad(GPIOE, GPIOE_PE05_SLEEP);//wakeup
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

  //~ palSetPadMode(GPIOC, GPIOC_PC07, PAL_MODE_OUTPUT_PUSHPULL);//SLEEP OK
  //~ palSetPad(GPIOC, GPIOC_PC07);

  //~ palSetPadMode(GPIOC, GPIOC_PC00, PAL_MODE_INPUT_ANALOG);//SLEEP OK
  //~ palSetPadMode(GPIOC, GPIOC_PC03, PAL_MODE_INPUT_ANALOG);//SLEEP OK
  //~ palSetPadMode(GPIOC, GPIOC_PC05, PAL_MODE_INPUT_ANALOG);//SLEEP OK

  pwmStart(&PWMD3, &pwm_bldc_cfg);

  adcStart(&ADCD1, NULL);

  palSetPadMode(GPIOB, GPIOB_PB10_GROUND_SENSORS_TX, PAL_MODE_OUTPUT_PUSHPULL);
  //~ palClearPad(GPIOB, GPIOB_PB10_GROUND_SENSORS_TX);

  /*
   * Creates the example thread.
   */
  //chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);

  serial_protocol_thread_init();


  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched.
   */
  while (true) {
    //~ compare_pads();
    //~ palTogglePort(GPIOB, 1<<GPIOB_PB10_GROUND_SENSORS_TX);
    /*
    char c=0;

    c=sdGetTimeout(&SD1,MS2ST(100));
    if (c=='s'){
      //~ test_pad();
      palSetPadMode(GPIOC, GPIOC_PC07, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPad(GPIOC, GPIOC_PC07);
      chThdSleepMilliseconds(1000);
      palClearPad(GPIOC, GPIOC_PC07);
      palSetPadMode(GPIOC, GPIOC_PC07, PAL_MODE_INPUT);
    }else if(c=='p'){
      palTogglePad(GPIOE, GPIOE_PE05_SLEEP);
      chprintf(SERIAL,"sleep=%d\r\n",palReadPad(GPIOE, GPIOE_PE05_SLEEP));
    }else if(c=='m'){
      if(motor_en == 0){
        pwmEnableChannel(&PWMD3, 0, 3500/2);//tim3-ch1
        pwmEnableChannel(&PWMD3, 2, 3500/2);//tim3-ch3
        motor_en=1;
      }else{
        motor_en=0;
        pwmDisableChannel(&PWMD3, 0);//tim3-ch1
        pwmDisableChannel(&PWMD3, 2);//tim3-ch3
      }
    }else if(c=='b'){
      if(brush_en == 0){
        pwmEnableChannel(&PWMD3, 3, 3500/2);//tim3-ch1
        brush_en=1;
        palSetPadMode(GPIOB, GPIOB_PB14_TOURBINE_ENABLE, PAL_MODE_OUTPUT_PUSHPULL);//SLEEP OK
        palSetPad(GPIOB, GPIOB_PB14_TOURBINE_ENABLE);
      }else{
        brush_en=0;
        pwmDisableChannel(&PWMD3, 3);//tim3-ch1
        palClearPad(GPIOB, GPIOB_PB14_TOURBINE_ENABLE);
      }
    }
    */
    //else{
      chThdSleepMilliseconds(500);
    //~ }
  }
}
