/*
 * My_PID.H
 *
 *  Created on: 2021年7月19日
 *      Author: fenglimin1
 */

#ifndef INC_MYMC_PID_H_
#define INC_MYMC_PID_H_
#include "Mymc_type.h"


void currentPID_init();
float currentPID_Control(float SetValue,float ActValue);
#endif /* INC_MYMC_PID_H_ */
