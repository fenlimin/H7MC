/*
 * Mymc_math.h
 *
 *  Created on: Jul 15, 2021
 *      Author: fenglimin1
 */

#ifndef INC_MYMC_MATH_H_
#define INC_MYMC_MATH_H_

#include "Mymc_type.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_adc.h"
#include "stm32h7xx_ll_tim.h"

#include "stm32h7xx_hal_adc.h"

#define SQRT_2  1.4142
#define SQRT_3  1.732
#define divSQRT_3 (int32_t)0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315*/
#define ADV_TIM_CLK_MHz  240
#define DEADTIME_NS (800)
#define DTcompension  (uint16_t)((DEADTIME_NS * ADV_TIM_CLK_MHz) / 2000)
#define PWM_FREQUENCY   20000
#define pi  			(float)3.141592
#define PWM_MAX_TON                 (0.95f)
#define SECTOR_1  1u
#define SECTOR_2  2u
#define SECTOR_3  3u
#define SECTOR_4  4u
#define SECTOR_5  5u
#define SECTOR_6  6u
#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

typedef struct
{
  float hCos;
  float hSin;
} Trig_Components_t;

struct PWMC_Handle
{

  uint16_t  hT_Sqrt3;                                    /**< a constant utilized by PWM algorithm (@f$\sqrt{3}@f$) */
  uint16_t  CntPhA;                                     /**< PWM Duty cycle for phase A */
  uint16_t  CntPhB;                                     /**< PWM Duty cycle for phase B */
  uint16_t  CntPhC;                                     /**< PWM Duty cycle for phase C */
  uint16_t  SWerror;                                     /**< Contains status about SW error */
  uint8_t   Sector;                                     /**< the space vector sector number */
  uint16_t  lowDuty;
  uint16_t  midDuty;
  uint16_t  highDuty;
  bool TurnOnLowSidesAction;                            /**< true if TurnOnLowSides action is active,
                                                              false otherwise. */
  uint16_t  OffCalibrWaitTimeCounter;                   /**< Counter to wait fixed time before motor
                                                              current measurement offset calibration. */
  uint8_t   Motor;                                      /**< Motor reference number */
  bool      RLDetectionMode;                             /**< true if enabled, false if disabled. */
  int16_t   Ia;                                         /**< Last @f$I_{A}@f$ measurement. */
  int16_t   Ib;                                         /**< Last @f$I_{B}@f$ measurement. */
  int16_t   Ic;                                         /**< Last @f$I_{C}@f$ measurement. */
  uint16_t  DTTest;                                      /**< Reserved */

  /* former  PWMnCurrFdbkParams_t */
  uint16_t PWMperiod;                                   /**< PWM period expressed in timer clock cycles unit:
                                                           *  @f$hPWMPeriod = TimerFreq_{CLK} / F_{PWM}@f$    */
  uint16_t OffCalibrWaitTicks;                          /**< Wait time duration before current reading
                                                           *  calibration expressed in number of calls
                                                           *  of PWMC_CurrentReadingCalibr() with action
                                                           *  #CRC_EXEC */
  uint16_t DTCompCnt;                                   /**< Half of Dead time expressed
                                                           *  in timer clock cycles unit:
                                                           *  @f$hDTCompCnt = (DT_s \cdot TimerFreq_{CLK})/2@f$ */
  uint16_t  Ton;                                         /**< Reserved */
  uint16_t  Toff;                                        /**< Reserved */

};

#define START_INDEX 63
#define MAX_MODULE 32767
#define CIRCLE_LIMIT_TABLE {\
32767,32390,32146,31907,31673,31444,31220,31001,30787,30577,30371,\
30169,29971,29777,29587,29400,29217,29037,28861,28687,28517,\
28350,28185,28024,27865,27709,27555,27404,27256,27110,26966,\
26824,26685,26548,26413,26280,26149,26019,25892,25767,25643,\
25521,25401,25283,25166,25051,24937,24825,24715,24606,24498,\
24392,24287,24183,24081,23980,23880,23782,23684,23588,23493,\
23400,23307,23215,23125\
}



alphabeta_t Clarke(abc_t Iabc);
qd_t Park( alphabeta_t Ialphabeta,float hElAngle);
alphabeta_t Rev_Park(qd_t Vqd,float hElAngle );
Trig_Components_t Trig_Functions(int16_t hElAngle   );
void CAL_SVPWM( qd_t Vdq,float hElAngle );
qd_t Circle_Limitation( qd_t Vqd );
abc_t VDQToVABC(qd_t Vqd,float hElAngle );
void currentReadingCalibration();
abc_t Get_Phase_Currents();
abc_t Get_Phase_ZeroCurrents();
float *Ia,*Ib,*Ic,*Va,*Vb,*Vc,*Ialpha,*Ibeta,*Valpha,*Vbeta,*Iq,*Id,*Vq,*Vd;
/* Exported functions ------------------------------------------------------- */



#endif /* INC_MYMC_MATH_H_ */
