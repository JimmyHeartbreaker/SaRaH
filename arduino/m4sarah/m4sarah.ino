
//#pragma GCC optimize ("O3,inline-functions,fast-math") 
#define CORE_CM4
#include <Arduino.h>
#include "headers\rpc_mpi.h"
#include "headers\direction_commands.h"
#include <string>
#ifndef IRAM_ATTR
#define IRAM_ATTR __attribute__((section(".ramfunc")))
#endif



char R_STOP = 0x07;
char R_TURN_LEFT = 0x08;
char R_TURN_RIGHT = 0x09;
char R_REVERSE = 0x10;
char R_FORWARD = 0x11;

struct pid_state
{
	double kp;
	double ki;
	double kd;
	double max;
	double min;
	double deadband;
	double integral; 
	double prev_error;
};

struct Encoder
{
  volatile int count;
  volatile int total_count;  
  int pin;
  std::string name;
  double prev_speed;
  double new_speed;
};


  pid_state pid_state_l = {
                          0.05, //kp
                          0,    //ki
                          0,    //kd
                          +1000,
                          -1000,
                          0, 
                          0,
                          0};
  double left_pwm;


  pid_state pid_state_r  = {
                          0.05, //kp
                          0,    //ki
                          0,    //kd
                          +1000,
                          -1000,
                          0, 
                          0,
                          0};
double right_pwm;

Encoder FL = {0,0,D0,"FL",0,0};
Encoder BL = {0,0,D1,"BL",0,0};
Encoder FR = {0,0,D2,"FR",0,0};
Encoder BR = {0,0,D3,"BR",0,0};

Encoder* encoders[4];

int HB_L_FWD_PWM_pin = D6;
int HB_L_BCK_PWM_pin = D7;
int HB_L_PWM_PIN = HB_L_FWD_PWM_pin;
int HB_L_PWM_PIN_next = HB_L_FWD_PWM_pin;


int HB_R_FWD_PWM_pin = D9;
int HB_R_BCK_PWM_pin = D10;
int HB_R_PWM_PIN = HB_R_FWD_PWM_pin;
int HB_R_PWM_PIN_next = HB_R_FWD_PWM_pin;



float left_command = 0.0;
float right_command = 0.0;


float next_left_command = 0.0;
float next_right_command = 0.0;

void stop_handler(RPC_MPI::Message& m)
{
    left_command = 0;
    right_command=0;
    next_left_command = -1;    
    next_right_command = -1; 
 
}
void forward_handler(RPC_MPI::Message& m)
{ 
    left_command = 0;
    right_command=0;
    next_left_command = 150;    
    next_right_command = 150; 
    HB_R_PWM_PIN_next = HB_R_FWD_PWM_pin;    
    HB_L_PWM_PIN_next = HB_L_FWD_PWM_pin;
}
void rotate_left_handler(RPC_MPI::Message& m)
{ 
    left_command = 0;
    right_command=0;
    next_left_command = 150;    
    next_right_command = 150; 
    HB_R_PWM_PIN_next = HB_R_FWD_PWM_pin;    
    HB_L_PWM_PIN_next = HB_L_BCK_PWM_pin;
}

void rotate_right_handler(RPC_MPI::Message& m)
{
    left_command = 0;
    right_command=0;
    next_left_command = 150;    
    next_right_command = 150; 
    HB_R_PWM_PIN_next = HB_R_BCK_PWM_pin;    
    HB_L_PWM_PIN_next = HB_L_FWD_PWM_pin;
}

void reverse_handler(RPC_MPI::Message& m)
{ 
    left_command = 0;
    right_command=0;
    next_left_command = 150;    
    next_right_command = 150; 
    HB_R_PWM_PIN_next = HB_R_BCK_PWM_pin;    
    HB_L_PWM_PIN_next = HB_L_BCK_PWM_pin;
}

void serial_handler(RPC_MPI::Message& m)
{
  RPC_MPI::Print(m.data,m.length);
}


void IRAM_ATTR  ISR_FL() 
{
  FL.count++;
  FL.total_count++;
}

void IRAM_ATTR ISR_BL() 
{
  BL.count++;
  BL.total_count++;
}

void IRAM_ATTR ISR_FR() 
{
  FR.count++;
  FR.total_count++;
}

void IRAM_ATTR  ISR_BR() 
{
  BR.count++;
  BR.total_count++;
}

long prev_micros;

void setup() 
{ 

  encoders[0] = &FL;
  encoders[1] = &BL;
  encoders[2] = &FR;
  encoders[3] = &BR;

  RPC_MPI::Setup();
  RPC_MPI::RegisterMessageHandler(RPC_MPI::SERIAL_OUT,serial_handler);
  RPC_MPI::RegisterMessageHandler(R_STOP,stop_handler);
  RPC_MPI::RegisterMessageHandler(R_FORWARD,forward_handler);
  RPC_MPI::RegisterMessageHandler(R_REVERSE,reverse_handler);
  RPC_MPI::RegisterMessageHandler(R_TURN_LEFT,rotate_left_handler);
  RPC_MPI::RegisterMessageHandler(R_TURN_RIGHT,rotate_right_handler);

  RPC_MPI::Print("M4 setup starting...");
  pinMode(FL.pin, INPUT);   // or INPUT_PULLUP if needed
  pinMode(BL.pin, INPUT);
  pinMode(FR.pin, INPUT);
  pinMode(BR.pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(FL.pin), ISR_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR.pin), ISR_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(BL.pin), ISR_BL, RISING);
  attachInterrupt(digitalPinToInterrupt(BR.pin), ISR_BR, RISING);  

  prev_micros = micros();

  pinMode(HB_L_FWD_PWM_pin, OUTPUT);
  pinMode(HB_R_FWD_PWM_pin, OUTPUT);
  pinMode(HB_L_BCK_PWM_pin, OUTPUT);
  pinMode(HB_R_BCK_PWM_pin, OUTPUT);
  RPC_MPI::Print("M4 setup completed");
}

/// <summary>
/// consult uni notes
/// </summary>
/// <param name="y"></param>
/// <returns></returns>
double PIDUpdate(pid_state* pid, double error,double dt)
{
	if (fabs(error) <= pid->deadband)
	{
		error = 0.0;
	}
	double de_dt = (error - pid->prev_error) / dt;

	double	input_delta = 
                    pid->kp * error + 
                    pid->ki * pid->integral + 
                    pid->kd * de_dt;
	
	double new_input = std::fmax(pid->min, std::fmin(input_delta, pid->max));

	//	# anti - windup correction;
	pid->integral += dt * error;// + (new_input - (input_delta+prev_input)) / pid->Tt;

	pid->prev_error = error;
	return new_input;
}
double print_delay;
void loop()
{  
  long this_micros = micros();
  double dt = (this_micros - prev_micros ) / 1e6;
  
  
  RPC_MPI::ProcessMessages();

  if(dt<0.02)
    return;

    // if(print_delay > 1)
    // {
    //   print_delay-=1;

    //     for(int i =0;i<4;i++)
    //     {
    //       RPC_MPI::Print(encoders[i]->name);
    //       RPC_MPI::Print(" COUNT:");
    //       RPC_MPI::Print(encoders[i]->count);  
           
    //     } 

    // }
  
  print_delay += dt;
  for(int i =0;i<4;i++)
  {
      encoders[i]->prev_speed =  encoders[i]->new_speed;
      encoders[i]->new_speed = encoders[i]->count / dt;
      encoders[i]->count = 0;
  }



    double avg_left_speed = (FL.new_speed + BL.new_speed)/2;
    double avg_right_speed = (FR.new_speed + BR.new_speed)/2;
    
    if(avg_left_speed < 1 && avg_right_speed < 1)
    {
      if(HB_R_PWM_PIN != HB_R_PWM_PIN_next || HB_L_PWM_PIN != HB_L_PWM_PIN_next || left_command!= next_left_command || right_command != next_right_command)
      {
   //    RPC_MPI::Print("command updated");
        left_command = max(0,next_left_command);
        right_command = max(0,next_right_command);
        next_left_command = left_command;
        next_right_command = right_command;
        
        for(int i =0;i<4;i++)
        {
       //   RPC_MPI::Print("COUNT:");
          //RPC_MPI::Print(encoders[i]->total_count);  
           
           encoders[i]->total_count = 0;
        }
      }
        analogWrite(HB_R_PWM_PIN, (int)0);   
        analogWrite(HB_L_PWM_PIN, (int)0);
        HB_R_PWM_PIN = HB_R_PWM_PIN_next;
        HB_L_PWM_PIN = HB_L_PWM_PIN_next;
    }
    double average_total =0;
    for(int i =0;i<4;i++)
    {
      average_total += encoders[i]->total_count;
    }
    average_total /= 4;
    if(average_total > 1122)
    {
        left_command = 0;
        right_command=0;
        next_left_command = -1;    
        next_right_command = -1; 
        for(int i =0;i<4;i++)
        {
          encoders[i]->total_count=0;
        }
    }
    double left_error =  (left_command) - avg_left_speed ;
    double right_error =  (right_command) -  avg_right_speed;

    double l_inc = PIDUpdate(&pid_state_l,left_error,dt);
    double r_inc = PIDUpdate(&pid_state_r,right_error,dt);
    left_pwm +=  l_inc;
    left_pwm = std::fmax(0.0, std::fmin(left_pwm, 255.0));
    double left_pwm_out = left_pwm;
    if(left_pwm_out < 40)
    {
      left_pwm_out = 0;
    }
   

    right_pwm += r_inc; 
    
    right_pwm =std::fmax(0.0, std::fmin(right_pwm, 255.0));
       double right_pwm_out = right_pwm;
     if(right_pwm_out < 40)
    {
      right_pwm_out = 0;
    }

    
    analogWrite(HB_R_PWM_PIN, (int)right_pwm_out);
    analogWrite(HB_L_PWM_PIN, (int)left_pwm_out);

    


    prev_micros = this_micros;
}