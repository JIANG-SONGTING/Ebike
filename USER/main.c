#include "led.h"
#include "delay.h"
#include "sys.h"
#include "includes.h"
#include "TXXY.h"
#include "usart1.h"
#include "usart3.h"	  
#include "adc.h"
#include "24cxx.h" 
#include "myiic.h"
#include "mpu6050.h"
#include "mpuiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ServoDj.h" 
#include "stdlib.h" 
#include "usart2.h"	
#include "UsrCat1.h"
#include "gps.h"  
 
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
__align(8) static OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
 			   
//LED0任务
//设置任务优先级
#define LED0_TASK_PRIO       			7 
//设置任务堆栈大小
#define LED0_STK_SIZE  		    		1000
//任务堆栈	
__align(8) static OS_STK LED0_TASK_STK[LED0_STK_SIZE];
//任务函数
void led0_task(void *pdata);


//LED1任务
//设置任务优先级
#define LED1_TASK_PRIO       			6 
//设置任务堆栈大小
#define LED1_STK_SIZE  					364
//任务堆栈
__align(8) static OS_STK LED1_TASK_STK[LED1_STK_SIZE];
//任务函数
void led1_task(void *pdata);

//设置任务优先级
#define Test_TASK_PRIO       			6 
//设置任务堆栈大小
#define Test_STK_SIZE  					364
//任务堆栈
__align(8) static OS_STK Test_TASK_STK[Test_STK_SIZE];
//任务函数
void Test_task(void *pdata);


 int main(void)
 {	
	 u8 n=0;
	delay_init();	    	 //延时函数初始化	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	LBIO_Init();		  	//初始化与LED连接的硬件接口
	USART1_Init(9600); 
	USART3_Init(115200);  
	 
  USART2_Init(36,115200);// 初始化 
//  LinkInitSet(); //此函数 只使用一次就行，目的是 设置与平台 通信的域名 端口 序列号
	
	Adc_Init();
	AT24CXX_Init();
	AT24CXX_Write(0,0,10);	
	MPU_Init();					//初始化MPU6050
	while(mpu_dmp_init())  
	{
	 delay_ms(50);
	 if(n++>10)break;
		
	}
  DuoJi_Init();	//PWM
	delay_ms(1000);delay_ms(1000);
	PraInit();//迪文参数初始化
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	
 }

	  
//开始任务
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
  	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
// 	OSTaskCreate(led0_task,(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);
	OSTaskCreate(Test_task,(void *)0,(OS_STK*)&Test_TASK_STK[Test_STK_SIZE-1],Test_TASK_PRIO);
 	OSTaskCreate(led1_task,(void *)0,(OS_STK*)&LED1_TASK_STK[LED1_STK_SIZE-1],LED1_TASK_PRIO);	 				   
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
u8 CHfg[6]={0,0,0,0,0,0};
float Vfg[7]={0,0,0,0,0,0,0};
u8 Infg[6]={0,0,0,0,0,0};
float XYZ[3];
u8 PwmT=0;
float JD,WD;
//LED0任务
void led0_task(void *pdata)
{	 	
  u8 cmd,cmd1,t,ch,i=0,Tcnt ;
	  
  //上网 使用
  char Pstr[600];
	u8 Dbuf[20]; 
	float RevSetD=0; 
   
	u16 Cnt=0,fg=0;
	
	u8 PwmLit=T_Pra[0];
	 while(1)
	{
		if(USART2_RX_STA&0x8000)
		{		 
		
		     fg=0;
	      	if(GetRevDat(&USART2_RX_BUF[10],1,Dbuf)==1)//接收到 "flag":"3"
					{
					      RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
						    if(RevSetD>0)cmd1=1;
						    else cmd1=0;
						    CHfg[0]=cmd1; 
                fg=1;
					} 
				  else if(GetRevDat(&USART2_RX_BUF[10],2,Dbuf)==1)//接收到 "flag":"4"
					{
					      RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
                if(RevSetD>0)cmd1=1;
						    else cmd1=0;
						    CHfg[1]=cmd1;
								fg=1;							   
					} 
				  else if(GetRevDat(&USART2_RX_BUF[10],3,Dbuf)==1)//接收到 "flag":"4"
					{
					      RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
                if(RevSetD>0)cmd1=1;
						    else cmd1=0;
						    CHfg[2]=cmd1;
								fg=1;							   
					} 
				  else if(GetRevDat(&USART2_RX_BUF[10],4,Dbuf)==1)//接收到 "flag":"4"
					{
					      RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
                if(RevSetD>0)cmd1=1;
						    else cmd1=0;
						    CHfg[3]=cmd1;
								fg=1;							   
					} 
				  else if(GetRevDat(&USART2_RX_BUF[10],5,Dbuf)==1)//接收到 "flag":"4"
					{
					      RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
                if(RevSetD>0)cmd1=1;
						    else cmd1=0;
						    CHfg[4]=cmd1;
								fg=1;							   
					} 
				  else if(GetRevDat(&USART2_RX_BUF[10],6,Dbuf)==1)//接收到 "flag":"4"
					{
					      RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
                if(RevSetD>0)cmd1=1;
						    else cmd1=0;
						    CHfg[5]=cmd1;
								fg=1;							   
					} 
				  else if(GetRevDat(&USART2_RX_BUF[10],7,Dbuf)==1)//接收到 "flag":"4"
					{
					    //  RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
                snprintf((void*)WbAT.str,10,"%s",Dbuf);
							  DatStr_Db(WbAT.adr,WbAT.str);
							  AT24CXX_Write(0,(u8*)WbAT.str,10);
						    fg=1;							   
					} 
				  else if(GetRevDat(&USART2_RX_BUF[10],8,Dbuf)==1)//接收到 "flag":"4"
					{
					      RevSetD=atof((void*)Dbuf);//把字符串转换成整型数的一个函数   获取到返回的数据 
							  T_Pra[0]=RevSetD;
						    PwmLit=T_Pra[0];
						    AT24CXX_Write(20,(u8*)T_Pra,10); //	数据// 保存结构体数据   
                fg=1;						
					}  
			 if(fg)	
			 {				 
				 Cnt=1900;
				 memset(USART2_RX_BUF,0,USART2_MAX_RECV_LEN);
			 }
		   USART2_RX_STA=0;
		}	
		if(Cnt++>200)
		{
		  Cnt=0; 
			sprintf((void*)Pstr,"{\"sensorDatas\":[{\"value\":%d},{\"value\":%.1f},{\"value\":%d},{\"value\":%d},{\"value\":%.1f},{\"value\":%d},{\"value\":%d},{\"value\":%.1f},{\"value\":%d},\
				                                     {\"value\":%d},{\"value\":%.1f},{\"value\":%d},{\"value\":%d},{\"value\":%.1f},{\"value\":%d},{\"value\":%d},{\"value\":%.1f},{\"value\":%d},\
			                                       {\"value\":%.1f},{\"value\":%.1f},{\"value\":%.1f},{\"str\":\"%s\"},{\"value\":%.1f},{\"value\":%d},{\"value\":%d},{\"lat\":%.5f,\"lng\":%.5f}]}"\
			,CHfg[0],Vfg[0],Infg[0],CHfg[1],Vfg[1],Infg[1],CHfg[2],Vfg[2],Infg[2],CHfg[3],Vfg[3],Infg[3],CHfg[4],Vfg[4],Infg[4],CHfg[5],Vfg[5],Infg[5],XYZ[0],XYZ[1],XYZ[2],WbAT.str,Vfg[6],PwmT,PwmLit,	WD,JD );
		  u2_printf((void*)Pstr);
			delay_ms(50);
	    USART2_RX_STA=0;
		} 
		
		if(GPS_IO==0)
		{		 
				if(USART1_RX_STA&0X8000)		//判断是否成功 接收到一次数据了
				{  
					GPS_Analysis(&gpsinfo,(u8*)USART1_RX_BUF);//分析字符串 分析接收到的 
					JD=(float)gpsinfo.longitude/100000; 
					WD=(float)gpsinfo.latitude/100000;   
					USART1_RX_STA=0;  
				}  
//				USART1_printf("GPS=%s",USART1_RX_BUF);
//				delay_ms(1000);
		}
		else 
		{  
//			 USART1_printf("PTA=%s",USART1_RX_BUF);
//			 delay_ms(1000);
				if(USART1_RX_STA&0x8000)
				{		
						//USART1_printf("USART1_RX_BUF=%s",USART1_RX_BUF);	
						snprintf((void*)WbCmd.str,40,"%s",USART1_RX_BUF);
						for(i=strlen((void*)WbCmd.str);i<40;i++) WbCmd.str[i]=' ';
						DatStr_Db(WbCmd.adr,WbCmd.str);
						t=RevStrJX(USART1_RX_BUF,USART1_RX_STA&0x0fff);//数据解析
						if(t>1)
						{
							 cmd=atoi((void*)RevList[0]);
							 switch(cmd)
							 {
								 case 0://设置密码
										snprintf((void*)WbAT.str,10,"%s",RevList[1]);
										DatStr_Db(WbAT.adr,WbAT.str);
										AT24CXX_Write(0,(u8*)WbAT.str,10);
								 break;
								 case 1:
								 case 2: 
								 case 3:
								 case 4: 
								 case 5:
								 case 6://设置 通道1-6
									 cmd1=atoi((void*)RevList[1]);
									 ch=cmd-1;						 
									 if(cmd1==0||cmd1==1)//设置通道 为0//设置通道 为1
									 {  CHfg[ch]=cmd1; 
									 } 
									 else if(cmd1==2)//读取通道状态
									 {
											 printf("%d,%.1f,%d,",cmd,Vfg[ch],Infg[ch]);
									 }  
								 break;
							 case 9://设置 通道1-6
									 cmd1=atoi((void*)RevList[1]);
									 
													 
									 if(cmd1==0||cmd1==1)//设置通道 为0  //设置通道 为1
									 {  
											 for(i=0;i<6;i++)	CHfg[i]=cmd1;
									 } 
									 else if(cmd1==2)//读取通道状态
									 {  
											 for(i=0;i<6;i++)
											 printf("%d,%.1f,%d,\r\n",i+1,Vfg[i],Infg[i]);
									 }  
								 break;
							 } 
						}  
						USART1_RX_STA=0;
				}	 
	 }
		
		
		if(Tcnt++>100)
		{
			 
		  Tcnt=0;
			CH1(CHfg[0]); CH2(CHfg[1]); CH3(CHfg[2]); CH4(CHfg[3]); CH5(CHfg[4]); CH6(CHfg[5]);  
      for(i=0;i<7;i++) 
			{
			  if(i==2)Vfg[i]=Get_Adc(8)/4095.0*3.3;
				else if(i==3)Vfg[i]=Get_Adc(9)/4095.0*3.3;
				else 	Vfg[i]=Get_Adc(i)/4095.0*3.3;
			} 
			
		  Infg[0]=IN1_IO;Infg[1]=IN2_IO;Infg[2]=IN3_IO;Infg[3]=IN4_IO;Infg[4]=IN5_IO;Infg[5]=IN6_IO;
			
			PwmT=Vfg[6]/3.0*100;
		  if(PwmT>99)PwmT=99;
			if(PwmT<=0)PwmT=1;
			if(PwmT>PwmLit)PwmT=PwmLit;
			
			TIM_SetCompare2(TIM3,PwmT/100.0*PwmDat); 
		  DataRef();	
		} 
		 
	  delay_ms(10);
	};
}

//LED1任务
void led1_task(void *pdata)
{	  
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	
 
	while(1)
	{  
		 delay_ms(10); 
		 if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			
			XYZ[0]=pitch;//X
			XYZ[1]=yaw;//y
			XYZ[2]=roll;//z
			
//	    printf("aacx:%6d aacy:%6d aacz:%6d gyrox:%6d gyroy:%6d gyroz:%6d roll:%6d pitch:%6d yaw:%6d \r\n",aacx,aacy,aacz,gyrox,gyroy,gyroz ,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
       
		}
	    
	 };
}

void Test_task(void *pdata)
{	  
 u8 TEST_A=0x01;
 u8 TEST_B=0x25;
	
	while(1)
	{  
		
		 test_send_packet(TEST_A,TEST_B); 
		 delay_ms(5000);
		 TEST_A++;
		 TEST_B++;
	};
	    
	 
}



