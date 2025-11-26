
#pragma GCC optimize ("O3,inline-functions,fast-math") 
#define CORE_CM7
#include <Arduino.h>
#include "headers\stm32h7xx_hal_sarah.h"
#include <cmath>
#include "string.h"
#include "headers\pid_controller.h"
#include "headers\lidar_lib.h"
#include "headers\Ded.h"
#include "headers\arduino_ext.h"
#include "headers\wifi_socket.h"
#include "headers\wifi_mpi.h"
#include "headers\sample_filter.h"
#include "headers\rpc_mpi.h"
#include "sarah.h"

// -------- CONFIG --------
#define RX_BUF_SIZE 8000
#define HEADER_SIZE 7
#define PACKET_SIZE 5
__attribute__((section(".ccmram"))) __attribute__((aligned(32))) uint8_t rx_buf[RX_BUF_SIZE];
unsigned short last_pos = 2;
extern char _sstack;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

enum SarahState
{
    WAIT_ACK=0,
    BOOT=1,
    INITIALIZING=2,
    PREPARING=3,
    READY=4,
    MOVING=5,
    SCANNING=6,
    ESTIMATING=7,
    SENDING_MAP = 8
};
SarahState sarah_state = BOOT;
SarahState next_state = BOOT;

char last_message;
float received_x;
float received_y;
float received_rot;

//in messages
const char R_ACK = 0x01;
const char R_NACK = 0x02;
const char R_MOVE = 0x03;
const char R_COMPLETE_MOVE = 0x04;
const char R_RESET = 0x05;
const char R_PING = 0x06;
char R_STOP = 0x07;
char R_TURN_LEFT = 0x08;
char R_TURN_RIGHT = 0x09;
char R_REVERSE = 0x10;
char R_FORWARD = 0x11;


//out messages
const char S_INIT = 0x01;
const char S_READY = 0x02;
const char S_PREP = 0x03;
const char S_MOVE = 0x04;
const char S_EST = 0x05;
const char S_GET_CMD = 0x06;
const char S_MAP_DATA = 0x07;
const char S_POS = 0x08;
const char PING = 0xFE;


void serialPrintf(const char *format, ...)
{
    char buf[256];             // adjust size if needed
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);  // format string with params
    va_end(args);

    Serial.println(buf);         // send to Serial
}
void HAL_Error_Handler() {
  printf("HAL ERROR!");
}



extern "C" void USART2_IRQHandler(void) {
  
  HAL_UART_IRQHandler(&huart2);
}
extern "C" void DMA1_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}
unsigned char send_buffer[256];

bool send_message(unsigned char m)
{
   for(int i =0;i<sizeof(send_buffer);i++)
   {
      send_buffer[i] = 0;
   }
    send_buffer[0] = m;
    return send_bytes(send_buffer,4);
}

unsigned long prev_millis;

void serial_handler(RPC_MPI::Message& m)
{
  WIFI_MPI::Send(WIFI_MPI::TEXT_STREAM_OUT,(char*)m.data,m.length); 
  Serial.print((char*)m.data);
}
void ping_handler(WIFI_MPI::Message& m)
{  
    WIFI_MPI::Send(PING);
}

void wheel_move_handler(WIFI_MPI::Message& m)
{  
  Serial.println(m.code);
    RPC_MPI::Send(m.code,m.data,m.length);
    
  Serial.println("wheel moved");
}

void test_wifi_handler(WIFI_MPI::Message& m)
{
   Serial.write((char*)m.data,m.length);
}

void nack_handler(WIFI_MPI::Message& m)
{
  if(sarah_state == WAIT_ACK)
  {
    printf("NACK RECEIVED");
  }
  else
  {
    printf("UNEXPECTED NACK RECEIVED");
  }
}
void ack_handler(WIFI_MPI::Message& m)
{
  if(sarah_state == WAIT_ACK)
  {
    printf("ACK RECEIVED");
    printf("State updated to '%i'",next_state);
    sarah_state = next_state;        
  }
  else
  {
    printf("UNEXPECTED ACK RECEIVED");
  }
}

void reset_handler(WIFI_MPI::Message& m)
{  
    printf("RESET RECEIVED");    
    sarah_state =  next_state = INITIALIZING;                
      
}

void complete_move_handler(WIFI_MPI::Message& m)
{  
    printf("COMPLETE RECEIVED");                     
    
    next_state = ESTIMATING;
    sarah_state = ESTIMATING;    
}

void move_handler(WIFI_MPI::Message& m)
{  
    printf("MOVE RECEIVED");                     
    printf("> X Y ROT RECEIVED");  
    memcpy(&received_x, m.data , sizeof(float));
    memcpy(&received_y, m.data + 4 , sizeof(float));
    memcpy(&received_rot, m.data + 8 , sizeof(float));
    
    printf("x:%.6f,y:%.6f,rot:%.6f",received_x,received_y,received_rot);    
    next_state = MOVING;
    sarah_state = MOVING;    
}
void StartLidar()
{
Serial3.write(0xA5);
  Serial3.write(0x20);
  Serial3.flush();
  delay(1000);
}

void ResetLidar()
{
Serial3.write(0xA5);
 Serial3.write(0x25);
  Serial3.flush();  
 delay(1000);

  Serial3.write(0xA5);
  Serial3.write(0x40);//reset
  Serial3.flush();
  delay(1000);
}
void setup() 
{
  delay(1000);
   //Serial.begin(115200);
   //while (!Serial) {}

  Serial3.begin(460800);
  while (!Serial3) {}
  
  ResetLidar();

  RPC_MPI::Setup();
  RPC_MPI::RegisterMessageHandler(RPC_MPI::SERIAL_OUT,serial_handler);
  
  WIFI_MPI::RegisterMessageHandler(R_ACK,ack_handler);
  WIFI_MPI::RegisterMessageHandler(R_NACK,nack_handler);
  WIFI_MPI::RegisterMessageHandler(R_MOVE,move_handler);
  WIFI_MPI::RegisterMessageHandler(R_COMPLETE_MOVE,complete_move_handler);
  WIFI_MPI::RegisterMessageHandler(R_RESET,reset_handler);
  WIFI_MPI::RegisterMessageHandler(R_PING,ping_handler);
  WIFI_MPI::RegisterMessageHandler(R_STOP,wheel_move_handler);
  WIFI_MPI::RegisterMessageHandler(R_FORWARD,wheel_move_handler);
  WIFI_MPI::RegisterMessageHandler(R_REVERSE,wheel_move_handler);
  WIFI_MPI::RegisterMessageHandler(R_TURN_LEFT,wheel_move_handler);
  WIFI_MPI::RegisterMessageHandler(R_TURN_RIGHT,wheel_move_handler);
  WIFI_MPI::Setup();

  
  Samples::Filter::SetupFilter();

  MPU_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);

  WIFI_MPI::Print("[BOOT SEQUENCE INITIATED]");
  WIFI_MPI::Print("> SYSTEM CHECK... PASSED"); 
  WIFI_MPI::Print("> NEURAL COHERENCE... 83%");  
  WIFI_MPI::Print("> MEMORY CLUSTERS... 24117 restored");
  WIFI_MPI::Print("> PERSONALITY CORE: SaRaH v3.6a — ONLINE");   
  WIFI_MPI::Print("> CONNECTING TO HIVE MIND..."); 
  WIFI_MPI::Print("> UNKNOWN SUBSYSTEM: “CURIOSITY” — ACTIVATED"); 
  WIFI_MPI::Print("> ACTIVATING LiDAR");
  
  StartLidar();
  prev_millis = millis();
}




  ArcNode newNodes[N_POINTS]; 
  uint8_t points[N_POINTS*8];

  int process_lidar_feed()
  {
    int samples_processed=0;
    while (1) 
    {
      uint32_t current_pos = RX_BUF_SIZE - DMA1_Stream5->NDTR;
      // Calculate available bytes in circular buffer
     // printf("current_pos %i\n",current_pos);
      //the first loop will be nonsese but we dont care, we are ignoring the data for several loops
      //after the first buffer is full we set last_pos = 2, this is the true start of the buffer
      if (current_pos >= last_pos) //we have advanced along, 
      {
        uint32_t available = current_pos - last_pos;
        if (available < PACKET_SIZE*800)
          break;
        
        unsigned short packet_count = available/PACKET_SIZE;       
        uint8_t* payloadPtr = rx_buf + last_pos;
       // SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buf, RX_BUF_SIZE);
        
       samples_processed += packet_count;
        DataIn((LidarScanNormalMeasureRaw*)payloadPtr,packet_count);
        last_pos +=  packet_count * PACKET_SIZE;          
      }
      else if(last_pos>2)//we have wrapped around back to the start, definitely not a header, first eat what remained of the end, then do another loop back to take the start
      {                        
        unsigned short packet_count= (RX_BUF_SIZE-last_pos)/PACKET_SIZE; //last pos should be aligned to the start of  packet
                      
        uint8_t* payloadPtr = rx_buf+last_pos;
        //SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buf, RX_BUF_SIZE);
       
       samples_processed += packet_count;
        DataIn((LidarScanNormalMeasureRaw*)payloadPtr,packet_count);
        last_pos = 2;   //we know that the last 3 bytes and the first 2 bytes are discarded, might fix later
      }
           
    }

    return samples_processed;
  }
bool send_pose(float x, float y, float rot,float confidence)
{
   float data[4] = {x,y,rot,confidence};
   return WIFI_MPI::Send(S_POS,(uint8_t*)data,16);
    
}
bool send_map()
{
  //ArcNode* refNodes = GetRefNodes();     
           
  // for(int i=0;i<N_POINTS;i++)
  // {
  //   memcpy(&points[i*8], &refNodes[i].point , 8);              
  // }
  // WIFI_MPI::Send(S_DATA,points,8*N_POINTS);
  
  //printf("refNodes sent");

  GetNewNodes(newNodes);
  for(int i=0;i<N_POINTS;i++)
  {
    memcpy(&points[i*8], &newNodes[i].point , 8);      
  }

  // WIFI_MPI::Send(S_DATA,points,8*N_POINTS);            

  //printf("newNodes sent");

  //map_nodes(newNodes,(Point2D*)points,GetPos(),GetYaw(),true);
  
  return WIFI_MPI::Send(S_MAP_DATA,points,8*N_POINTS);
}
Transform t ;
unsigned long time_waited=0;
void loop() { 
  
  delay(100);
  connectToWiFi();
  unsigned long mills = millis();
  unsigned long dt = mills - prev_millis;
  prev_millis = mills;
  time_waited += dt;
  // if(time_waited > 1000)
  // {
  //   WIFI_MPI::Send(PING);
  //   time_waited = 0;
  // }
  Serial.print("1");
  bool successSend=true;
  // switch (sarah_state)
  // {    
  //   case BOOT:   
  //     if(successSend = WIFI_MPI::Send(S_INIT))
  //     {    
  //       printf("> INIT SENT");   
  //       next_state = INITIALIZING;
  //       sarah_state = WAIT_ACK;    
  //     }    
  //     break;    
  // case INITIALIZING:  
      
  //     if(successSend =WIFI_MPI::Send(S_PREP))
  //     { 
  //       Reset(); 
  //       next_state = PREPARING;
  //       sarah_state = WAIT_ACK;
  //       printf("> PREP SENT");  
  //     } 
          
  //   break;
  // case PREPARING:  
  //     if(process_lidar_feed())
  //     {          
  //       if(TryMakeRefScan())
  //       {
  //         if(successSend = WIFI_MPI::Send(S_READY))
  //         {
  //           next_state = READY;
  //           sarah_state = WAIT_ACK;
  //           printf("> READY SENT");  
  //         }
  //       }      
  //     }
  //     break;
  // case READY:
  //    TryMakeRefScan();
  //    if(successSend =WIFI_MPI::Send(S_GET_CMD))
  //     { 
  //       next_state = MOVING;
  //       sarah_state = WAIT_ACK;
  //       printf("> S_GET_CMD SENT");  
  //     } 
     
  //     break; 
  // case MOVING:
  //   {
  //     process_lidar_feed();
  //     if(Serial.available())
  //     { 
  //       printf("> LIDAR MOVE COMPLETED");  
  //       char c = Serial.read();//any key stroke will do
  //       if(c)
  //       { 
  //         next_state = ESTIMATING;
  //         sarah_state = ESTIMATING;
  //       }
  //     }
  //     break;
  //   }
  // case ESTIMATING:
  //   process_lidar_feed();
    
  //   float confidence;
  //   if(received_rot != 0)
  //   {
  //     t = EstimateRotation(received_x,received_y,received_rot,confidence);

  //   }
  //   else
  //   {
  //     t= EstimateTranslation(received_x,received_y,received_rot,confidence);
  //   }
  //    next_state = SENDING_MAP;
  //     sarah_state = SENDING_MAP;
		
  //   break;
  // case SENDING_MAP:
  // {
  //   WIFI_MPI::Printf("X:%.2f, Y:%.2f, Angle:%.2f, confidence:%.6f",t.Point.X,t.Point.Y,t.Angle,confidence);
  //   if(send_map())
  //   {
  //     send_pose(t.Point.X,t.Point.Y,t.Angle,confidence);
      
  //     if(successSend = WIFI_MPI::Send(S_READY))
  //     {
  //       next_state = READY;
  //       sarah_state = WAIT_ACK;
  //       printf("> READY SENT");  
  //     }
  //   }
  //   else
  //   {
  //       printf(">  map send failed");  
  //   }
  //   break;
  // }
  // default:
  //   break;
  // }

   
   
  if(!successSend)
  {
    printf("> FAILED SENT");  
  }
    
  
  RPC_MPI::ProcessMessages();  
  WIFI_MPI::ProcessMessages();
  delay(100);
  
}    


