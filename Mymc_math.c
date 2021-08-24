/*
 * Mymc_math.c
 *
 *  Created on: Jul 15, 2021
 *      Author: fenglimin1
 */

/*************************************************************/

#include "Mymc_math.h"
#define SIN_COS_TABLE {\
    0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
    0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
    0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
    0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
    0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
    0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
    0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
    0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
    0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
    0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
    0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
    0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
    0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
    0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
    0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
    0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
    0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
    0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
    0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
    0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
    0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
    0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
    0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
    0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
    0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
    0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
    0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
    0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
    0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
    0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
    0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
    0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE}

#define SIN_MASK        0x0300u
#define U0_90           0x0200u
#define U90_180         0x0300u
#define U180_270        0x0000u
#define U270_360        0x0100u
#define PWM_PERIOD		6000
#define  Limit_Pluse_Min_Value  100
#define  Limit_Pluse_Max_Value   PWM_PERIOD
const int16_t hSin_Cos_Table[256]= SIN_COS_TABLE;
uint64_t PhaseAOffset=0;
uint64_t PhaseBOffset=0;
uint64_t PhaseCOffset=0;

void currentReadingCalibration()
{
	LL_ADC_ClearFlag_JEOC(ADC1);
	LL_ADC_ClearFlag_JEOC(ADC2);
	LL_ADC_DisableIT_JEOC(ADC1);
	LL_ADC_DisableIT_JEOC(ADC2);

	PhaseAOffset=0;
	PhaseBOffset=0;
    //LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);
 //   LL_ADC_INJ_SetTriggerSource(ADC2, LL_ADC_INJ_TRIG_SOFTWARE);

	while (LL_ADC_IsActiveFlag_ADRDY( ADC1 ) == 0u)
	{
		LL_ADC_Enable(  ADC1 );
	}
	while (LL_ADC_IsActiveFlag_ADRDY( ADC2 ) == 0u)
	{
		LL_ADC_Enable(  ADC2 );
	}
	LL_ADC_INJ_StartConversion(ADC1);
	LL_ADC_INJ_StartConversion(ADC2);
	for(uint8_t tempcounter=0;tempcounter<16;tempcounter++)
	{
		while(!LL_ADC_IsActiveFlag_JEOC(ADC1)) { }  //等待注入组转换完成
		while(!LL_ADC_IsActiveFlag_JEOC(ADC2)) { }

		PhaseAOffset += ADC1->JDR1;//
		PhaseBOffset += ADC2->JDR1;
		//PhaseCOffset +=	ADC1->JDR2;//

		//LL_ADC_ClearFlag_JEOS(ADC1);
		LL_ADC_ClearFlag_JEOC(ADC1);
		LL_ADC_ClearFlag_JEOC(ADC2);
		LL_ADC_INJ_StartConversion(ADC1);
		LL_ADC_INJ_StartConversion(ADC2);
	}
	LL_ADC_INJ_StopConversion(ADC1);
	LL_ADC_INJ_StopConversion(ADC2);
	PhaseAOffset=PhaseAOffset/16;//平均值
	PhaseBOffset=PhaseBOffset/16;
	//PhaseCOffset=PhaseCOffset/16;
	LL_ADC_ClearFlag_JEOC(ADC1);
	LL_ADC_ClearFlag_JEOC(ADC2);
	//LL_TIM_CC_DisableChannel(TIM1, TIMxCCER_MASK_CH123);
}

abc_t Get_Phase_Currents()
{
	abc_t Iabc;
	int16_t wAux1, wAux2;
	while(!LL_ADC_IsActiveFlag_JEOC(ADC1)) { }  //等待注入组转换完成
	while(!LL_ADC_IsActiveFlag_JEOC(ADC2)) { }
//   wAux1=(int16_t)(ADC1->JDR1)-(int16_t)((int16_t)PhaseAOffset);
//   wAux2=(int16_t)(ADC2->JDR1)-(int16_t)((int16_t)PhaseBOffset);

	wAux1= (uint16_t)(ADC1->JDR1)-(uint16_t)33199;
	wAux2=(uint16_t)(ADC2->JDR1)-(uint16_t)33142;

	//wAux1=(int16_t)(ADC1->JDR1)-(int16_t)32767;
  // wAux2=(int16_t)(ADC2->JDR1)-(int16_t)32767;
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
	Iabc.a=(float)wAux1/32768*3.3/0.016;
	Iabc.b=(float)wAux2/32768*3.3/0.016;

	return Iabc;
}

/*******************************************************************************
* Function Name  : Clarke
* Description    : Clarke变换
*                alpha = a
*                beta = (2*b+a)/sqrt(3)
* Input          : U相(a轴)，V相(b轴)的电流值(float格式)
* Output         : alpha轴,beta轴的电流值(float格式)
*******************************************************************************/
alphabeta_t Clarke(abc_t Iabc)
{

	alphabeta_t Ialphabeta;
	float sqrt_3=SQRT_3;
	Ialphabeta.alpha=Iabc.a;
	Ialphabeta.beta=((Iabc.b)*2+(Iabc.a))/sqrt_3;
	return Ialphabeta;
}


/*******************************************************************************
* Function Name  : MCM_Park
* Description    : Park变换
*  d= alpha *cos(theta)+ beta *sin(Theta)
*  q= -alpha *sin(Theta)+ beta *cos(Theta)
* Input          : alpha轴,beta轴的电流值(float格式)，电角度(S16格式)
* Output         : Iq轴,Id轴的电流值(float格式)
*******************************************************************************/
qd_t Park( alphabeta_t Ialphabeta,float hElAngle)
{
	qd_t Iqd;
	Iqd.d=(Ialphabeta.alpha)*cos(hElAngle)+(Ialphabeta.beta)*sin(hElAngle);
	Iqd.q=-(Ialphabeta.alpha)*sin(hElAngle)+(Ialphabeta.beta)*cos(hElAngle);
	return Iqd;
}


/*******************************************************************************
* Function Name  : Rev_Park
* Description    : Park逆变换
*                  Valpha= Vd*Cos(theta)- Vq*Sin(theta)
*                  Vbeta=Vd*Sin(theta)+ Vq*Cos(theta)
* Input          : Vq,Vd(float格式)，电角度
* Output         : alpha轴,beta轴的电压值(float格式)
*******************************************************************************/
alphabeta_t Rev_Park(qd_t Vqd,float hElAngle )
{
	alphabeta_t Valphabeta;
	Valphabeta.alpha=(Vqd.d)*cos(hElAngle)-(Vqd.q)*sin(hElAngle);
	Valphabeta.beta=(Vqd.d)*sin(hElAngle)+(Vqd.q)*cos(hElAngle);
	return Valphabeta;
}
/*******************************************************************************
* Function Name  : VDQToVABC
* Description    : VDQ -> VABC
*Vq*1.154700538379252
*******************************************************************************/
abc_t VDQToVABC(qd_t Vqd,float hElAngle )
{
	abc_t Vabc;
	Vabc.a = Vqd.d * cos(hElAngle ) - (Vqd.q*1.154700538379252) * sin(hElAngle );
	Vabc.b = Vqd.d * cos(hElAngle  - (0.666667 * pi)) - (Vqd.q*1.154700538379252) * sin(hElAngle - (0.666667 * pi));
	Vabc.c = -Vabc.a - Vabc.b;
	return Vabc;

}

/*******************************************************************************
* Function Name  : Trig_Functions
* Description    : 本函数返回输入角度的cos和sin函数值
* Input          : angle in s16 format(0~180deg对应0~32768)
* Output         : Cosine and Sine 值
*******************************************************************************/
Trig_Components_t Trig_Functions( int16_t hAngle)
{
	  int32_t shindex;
	  uint16_t uhindex;
	//  int16_t hAngle=Angle_deg/180*32768;

	  Trig_Components_t Local_Components;

	  /* 10 bit index computation  */
	  shindex = ( ( int32_t )32768 + ( int32_t )hAngle );
	  uhindex = ( uint16_t )shindex;
	  uhindex /= ( uint16_t )64;

	  switch ( ( uint16_t )( uhindex ) & SIN_MASK )
	  {
	    case U0_90:
	      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( uhindex )];
	      Local_Components.hCos = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
	      break;

	    case U90_180:
	      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
	      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( uhindex )];
	      break;

	    case U180_270:
	      Local_Components.hSin = -hSin_Cos_Table[( uint8_t )( uhindex )];
	      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
	      break;

	    case U270_360:
	      Local_Components.hSin =  -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
	      Local_Components.hCos =  hSin_Cos_Table[( uint8_t )( uhindex )];
	      break;
	    default:
	      break;
	  }
	  Local_Components.hSin= Local_Components.hSin/32768;//转化为1单位
	  Local_Components.hCos= Local_Components.hCos/32768;
	  return ( Local_Components );
}



/*******************************************************************************
* Function Name  : CAL_SVPWM
* Description    : 计算占空比，用于发波
* Input          : alpha轴,beta轴的电压电流值(float格式)，
* Output         : 三相占空比
*******************************************************************************/
void CAL_SVPWM( qd_t Vqd,float hElAngle)
{
	abc_t Vabc;
    float Uu, Uv, Uw;
    float TuTmp, TvTmp, TwTmp;
    float UuvwMax, UuvwMin;
    float SymbUu, SymbUv, SymbUw;
    float UuvwCentral;
    float  OverModLarry[16] = {0.001221,0.003784,0.007813,0.012939,0.019897,\
    										0.029175,0.041748,0.060303,0.092407,0.137207,\
											0.188843,0.25061,0.327271,0.431519,0.636841,0.636841};
	float OverModRate,OverModRemainder;
	int OverModIndex;
	float Vvector = sqrt(Vqd.d * Vqd.d + Vqd.q * Vqd.q);
	if(Vvector<=1.157)
	{
		OverModRate = 0;
	}
	else if( Vvector >= 1.275)
	{
		OverModRate = 0.636841;
	}
	else
	{
		OverModIndex = floor((Vvector - 1.157) / 0.008);
		OverModRemainder = (Vvector - 1.157) - OverModIndex * 0.008;
		OverModRate = OverModLarry[OverModIndex-1] + ((OverModLarry[OverModIndex] - OverModLarry[OverModIndex-1]) / 0.008 * OverModRemainder);
	}
    Vabc=VDQToVABC(Vqd, hElAngle );
    Uu=Vabc.a;
    Uv=Vabc.b;
    Uw=Vabc.c;
    if(Uu >= Uv)
    {
        UuvwMax = Uu;
        UuvwMin = Uv;
        if(Uu <= Uw)
        {    UuvwMax = Uw; }
        else if(Uv >= Uw)
        {    UuvwMin = Uw; }
    }
    else
    {
        UuvwMax = Uv;
        UuvwMin = Uu;
        if(Uv <= Uw)
        {    UuvwMax = Uw; }
        else if(Uu >= Uw)
        {    UuvwMin = Uw; }
    }

    UuvwCentral = (0.5f) * (UuvwMax + UuvwMin);
    Uu = Uu - UuvwCentral;
    Uv = Uv - UuvwCentral;
    Uw = Uw - UuvwCentral;

    SymbUu = (Uu >= 0.0f)?(1.0f):(-1.0f);
    SymbUv = (Uv >= 0.0f)?(1.0f):(-1.0f);
    SymbUw = (Uw >= 0.0f)?(1.0f):(-1.0f);

    TuTmp = 1.0f - (0.5f) * (1.0f + Uu + SymbUu * OverModRate );    //- Axis->Acr.DtCompTu
    TvTmp = 1.0f - (0.5f) * (1.0f + Uv + SymbUv * OverModRate);    //- Axis->Acr.DtCompTv
    TwTmp = 1.0f - (0.5f) * (1.0f + Uw + SymbUw* OverModRate );    //- Axis->Acr.DtCompTw

    if(TuTmp >= PWM_MAX_TON)
        TuTmp = PWM_MAX_TON;
    if(TvTmp >= PWM_MAX_TON)
        TvTmp = PWM_MAX_TON;
    if(TwTmp >= PWM_MAX_TON)
        TwTmp = PWM_MAX_TON;

    if(TuTmp <= (1.0f - PWM_MAX_TON))
        TuTmp = (1.0f - PWM_MAX_TON);
    if(TvTmp <= (1.0f - PWM_MAX_TON))
        TvTmp = (1.0f - PWM_MAX_TON);
    if(TwTmp <= (1.0f - PWM_MAX_TON))
        TwTmp = (1.0f - PWM_MAX_TON);

    TIM1->CCR1 =(uint16_t)(TuTmp*PWM_PERIOD);
    TIM1->CCR2 =(uint16_t)(TvTmp*PWM_PERIOD);
    TIM1->CCR3 =(uint16_t)(TwTmp*PWM_PERIOD);

}


