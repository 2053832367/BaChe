#include "chassis_power_control.h"
#include "app_preference.h"
#include "tasks.h"

// float Power,Power1,powermax=90,k1=1.92e-07f;
float Power,Power1,powermax=100,k1=1.92e-07f;//֮ǰ�������Ϊ90
PowerClass::PowerClass()
{
	Power_K.K1=k1;
	Power_K.K2=POWER_K2;
	Power_K.constant=POWER_CONSTANT;
	cap_state=0;
  power_buffer_set=30;
  Power_K.Cap_power_open=CAP_POWER_OPEN;
  Power_K.Cap_power_close=CAP_POWER_CLOSE;
	
}
//int limit=30;
void PowerClass::Power_Feedback_Update()
{
	Power_limit.grade_power_limit=Message.robo->game_robot_state.chassis_power_limit;
  
	//Power_limit.Max_input_power=Power_limit.power_buffer_out+limit;//(Message.robo->game_robot_state.chassis_power_limit);//,最大功率更新，缓冲能量环输�?+该等级功率上�?
	Power_limit.Max_input_power=powermax;
	//，转速反�?
  Power_calc.chassis_speed_rpm[0]= Chassis.Motor[0].chassis_motor_measure->speed_rpm;
	Power_calc.chassis_speed_rpm[1]=Chassis.Motor[1].chassis_motor_measure->speed_rpm;
  Power_calc.chassis_speed_rpm[2]=Chassis.Motor[2].chassis_motor_measure->speed_rpm;
	Power_calc.chassis_speed_rpm[3]= Chassis.Motor[3].chassis_motor_measure->speed_rpm;
	
  Power_limit.Chassis_judge_power=Message.robo->power_heat_data.chassis_power;//，裁判系统功率，取出来看看罢
}
void PowerClass::Power_Calc()
{
	//,最大功率计�?
//	if(Message.SuperCapR.energy>5)//,百分比大�?5%时超部分功率
//	{
//	 if(cap_state==1)
//	 {
//		 Power_limit.Chassis_Max_power=Power_limit.Max_input_power+Power_K.Cap_power_open;//，开时大�?
//	 }
//	 else
//	 {
//     Power_limit.Chassis_Max_power=Power_limit.Max_input_power+Power_K.Cap_power_close;//,不开时小�?

//	 }
//	}
//	else
//	{
		Power_limit.Chassis_Max_power=Power_limit.Max_input_power;//，电容没电时不超
//	}
	//，计算当前功�?
	for(uint8_t i=0;i<4;i++)//�?
	{
    Power_calc.initial_give_power[i]//,当前电机功率模型计算
//	=TOQUE_COEFFICIENT*fabs(Power_calc.send_current_value[i])*fabs(Power_calc.chassis_speed_rpm[i])//,使用绝对值，防止有负功率
		=TOQUE_COEFFICIENT*Power_calc.send_current_value[i]*Power_calc.chassis_speed_rpm[i]//,力矩和转速（力矩使用电流发送值代替，因为其是线性关系）
		 +Power_K.K1*Power_calc.send_current_value[i]*Power_calc.send_current_value[i]//,力矩平方
		 +Power_K.K2*Power_calc.chassis_speed_rpm[i]*Power_calc.chassis_speed_rpm[i];//,转速平�?
		
		//,当前总功率（两轮加起来）
//		Power_calc.initial_total_power+=Power_calc.initial_give_power[i];//,当前总功率（两轮加起来）
	}
   if(Power_calc.initial_give_power<0)
			return ;
			Power_calc.initial_total_power=Power_calc.initial_give_power[0]+Power_calc.initial_give_power[1]
		                                +Power_calc.initial_give_power[2]+Power_calc.initial_give_power[3]+Power_K.constant;
	   
	//，做功率限制和分�?
	if(Power_calc.initial_total_power>Power_limit.Chassis_Max_power)//，超出最大功率限�?
	{
		float a,b,c,temp;
		float power_scale=Power_limit.Chassis_Max_power/Power_calc.initial_total_power;//，缩放系数计�?
	  for(uint8_t i=0;i<4;i++)
		{
      Power_calc.scaled_give_power[i]=Power_calc.initial_give_power[i]*power_scale;//,缩放后的最大功�?
			if(Power_calc.scaled_give_power<0)
				return ;
      Power_calc.scaled_total_power=Power_calc.scaled_give_power[0]+Power_calc.scaled_give_power[1]+Power_calc.scaled_give_power[2]+Power_calc.scaled_give_power[3];
			
			 a=Power_K.K1;//,二元一次方程的a
			 b=TOQUE_COEFFICIENT*Power_calc.chassis_speed_rpm[i];//，二元一次方程的b
			 c=Power_K.K2*Power_calc.chassis_speed_rpm[i]*Power_calc.chassis_speed_rpm[i]-Power_calc.scaled_give_power[i]+Power_K.constant;//，二元一次方程的c
		
						
			if(Power_calc.send_current_value[i]>0)//,向前走取正解
			{
				 temp=(-b+sqrt(b*b-4*a*c))/(2*a);//,二元一次方程的正解
				if(temp>16000)//，限制最大解
					Power_calc.send_current_value[i]=16000;
				else
					Power_calc.send_current_value[i]=temp;
			}
			else//,向后走取负解
			{
				 temp=(-b-sqrt(b*b-4*a*c))/(2*a);//,二元一次方程的负解
				if(temp<-16000)//，限制最大解
					Power_calc.send_current_value[i]=-16000;
				else
					Power_calc.send_current_value[i]=temp;		
			}
	  }
	}
}

