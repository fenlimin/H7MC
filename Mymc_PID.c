/*
 * My_PID.C
 *
 *  Created on: 2021年7月19日
 *      Author: fenglimin1
 */

#include "Mymc_PID.h"

#include "Mymc_math.h"

//位置型PID
struct _pid
{
    float SetValue;           //定义设定值
    float ActValue;           //定义实际值
    float err;                //定义偏差值
    float err_last;           //定义上一个偏差值
    float Kp,Ki,Kd;           //定义比例、积分、微分系数
    float integral;           //定义积分值
    float output;			  //定义输出
    float integralMax;
    float integralMin;
    float outMax;
    float outMin;
}pid;
void currentPID_init()
{
    pid.SetValue=0.0;
    pid.ActValue=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.integral=0.0;
    pid.Kp=0.1;//0.1
    pid.Ki=0.01;//0.001
    pid.Kd=0;
    pid.output=0;
    pid.integralMax=10;
    pid.integralMin=-10;
    pid.outMax=48;
    pid.outMin=-48;
}

float currentPID_Control(float SetValue,float ActValue)
{
    pid.SetValue=SetValue;
    pid.ActValue=ActValue;
    pid.err=pid.SetValue-pid.ActValue;
    pid.integral+=pid.err;
    pid.err_last=pid.err;//更新误差项
    if(pid.integral>pid.integralMax)
    {
    	pid.integral=pid.integralMax;
    }
    if(pid.integral<pid.integralMin)
    {
    	pid.integral=pid.integralMin;
    }

    pid.output=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
    if(pid.output>pid.outMax)
    {
    	pid.output=pid.outMax;
    }
    if(pid.output<pid.outMin)
    {
    	pid.output=pid.outMin;
    }
    return pid.output;
}

