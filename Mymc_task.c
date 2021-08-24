/*
 * Mymc_task.c
 *
 *  Created on: Jul 15, 2021
 *      Author: fenglimin1
 */
/***************************************************/
#include "Mymc_task.h"

/***********************************************/
#define POLE_PAIR_NUM 5
#define Forward_rotation 1//定义方向
extern uint64_t PhaseAOffset;
extern uint64_t PhaseBOffset;
float printValue1=0;
float printValue2=0;
float printValue3=0;
float printValue4=0;
float printValue5=0;
float printValue6=0;
float Iq_ref=0;
float Id_ref=5;
float hElAngle=0;//
float deltaTheta=0.000*pi;//每周期变化角度，每1单位对应18rpm(s16)//
volatile uint8_t loopFlag=1;
/****************************************************/
void FOC_openloop(void)
{
	qd_t Vqd;	abc_t Iabc;
	Iabc=Get_Phase_Currents(); //
	printValue1=Iabc.a;
	printValue2=Iabc.b;
	printValue3=PhaseAOffset;
	printValue4=PhaseBOffset;
	printValue5=ADC1->JDR1;//
	printValue6=ADC2->JDR1;
	if (Forward_rotation)
		  {
			  hElAngle=hElAngle+deltaTheta;
			       if(hElAngle>pi-deltaTheta)
			       {
			    	  hElAngle=-pi;
			       }
		  }
		  else
		  {
			  hElAngle=hElAngle-deltaTheta;
		       if(hElAngle>pi-deltaTheta)
		       {
		    	  hElAngle=-pi;
		       }
		  }
	// Rev_Park( hElAngle);
	  Vqd.q=0.05;
	  Vqd.d=0;
	CAL_SVPWM(Vqd,hElAngle);

}


void FOC_closeloop(void)
{
	abc_t Iabc;
	alphabeta_t Ialphabeta;
	qd_t Iqd,Vqd;
	Iabc=Get_Phase_Currents(); //采样ab相电流
	Ialphabeta=Clarke(Iabc);
	if (Forward_rotation)
		  {
			  hElAngle=hElAngle+deltaTheta;
			       if(hElAngle>pi-deltaTheta)
			       {
			    	  hElAngle=-pi;
			       }
		  }
		  else
		  {
			   hElAngle=hElAngle-deltaTheta;
		       if(hElAngle>pi-deltaTheta)
		       {
		    	  hElAngle=-pi;
		       }
		  }
	Iqd=Park(Ialphabeta,hElAngle);
	Vqd.q=currentPID_Control(Iq_ref,Iqd.q);
	Vqd.d=currentPID_Control(Id_ref,Iqd.d);
	printValue1=Iabc.a;
	printValue2=Iabc.b;
	printValue3=PhaseAOffset;
	printValue4=PhaseBOffset;
	printValue5=ADC1->JDR1;//
	printValue6=ADC2->JDR1;
	CAL_SVPWM(Vqd,hElAngle);

}
