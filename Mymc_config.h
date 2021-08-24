/*
 * My_config.h
 *
 *  Created on: 2021年7月19日
 *      Author: fenglimin1
 */

#ifndef INC_MYMC_CONFIG_H_
#define INC_MYMC_CONFIG_H_

#include "Mymc_math.h"
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT        0//2048
#define PID_TORQUE_KI_DEFAULT         0//468
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           100
#define PID_FLUX_KI_DEFAULT           0//468
#define PID_FLUX_KD_DEFAULT           100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      2048
#define TF_KIDIV                      16384
#define TF_KDDIV                      8192

#define TF_KPDIV_LOG                  LOG2(2048)
#define TF_KIDIV_LOG                  LOG2(16384)
#define TF_KDDIV_LOG                  LOG2(8192)
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ       1000 /*!<Execution rate of speed
                                                    regulation loop (Hz) */
#define _RPM                          60
#define SPEED_UNIT                    10
#define PID_SPEED_KP_DEFAULT          3620/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          2595/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      64
#define SP_KIDIV                      16384
#define SP_KDDIV                      16

#define SP_KPDIV_LOG                  LOG2(64)
#define SP_KIDIV_LOG                  LOG2(16384)
#define SP_KDDIV_LOG                  LOG2(16)

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          16042

/* Default settings */

#define DEFAULT_TARGET_SPEED_RPM      1440
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

#endif /* INC_MYMC_CONFIG_H_ */
