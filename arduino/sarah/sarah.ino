
#pragma GCC optimize ("O3") 
#include <Arduino.h>
#include "headers\stm32h7xx_hal_sarah.h"
#include <cmath>
#include "string.h"
#include "headers\pid_controller.h"
#include "headers\lidar_lib.h"
#include "headers\Ded.h"
#include "headers\arduino_ext.h"
#include "headers\lucas_client.h"
#include "headers\sample_filter.h"
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
    WAIT_ACK,
    BOOT,
    INITIALIZING,
    PREPARING,
    READY,
    BEGIN_MOVE,
    MOVING,
    SCANNING,
    ESTIMATING
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

//out messages
const char S_INIT = 0x01;
const char S_READY = 0x02;
const char S_PREP = 0x03;
const char S_MOVE = 0x04;
const char S_EST = 0x05;
const char S_GET_CMD = 0x06;
const char S_DATA = 0x07;


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
void setup() 
{
  delay(3000);
  Serial.begin(115200);
  while (!Serial) {}
  printf("[BOOT SEQUENCE INITIATED]"); Serial.flush();

  Serial3.begin(460800);
  while (!Serial3) {}
 
  Serial3.write(0xA5);
  Serial3.write(0x25);
  Serial3.flush();
  
  Serial3.write(0xA5);
  Serial3.write(0x40);//reset
  Serial3.flush();
  delay(2000);
  
  printf("> SYSTEM CHECK... PASSED"); Serial.flush();
  MPU_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);
  delay(100);
    
  printf("> NEURAL COHERENCE... 83%"); 
 
  delay(100);
  printf("> MEMORY CLUSTERS... 24117 restored");
  delay(100);
  printf("> PERSONALITY CORE: SaRaH v3.6a — ONLINE"); 
  delay(100);
  printf("> CONNECTING TO HIVE MIND..."); Serial.flush();
  setup_lucas_client();
  tryConnectToServer();
  Samples::Filter::SetupFilter();
 
  delay(100);
  printf("> UNKNOWN SUBSYSTEM: “CURIOSITY” — ACTIVATED"); 

  printf("> ACTIVATING LiDAR"); Serial.flush();
  
  Serial3.write(0xA5);
  Serial3.write(0x20);
  Serial3.flush();
  prev_millis = millis();
}



  ArcNode newNodes[N_POINTS]; 
  uint8_t points[N_POINTS*8+4];

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
void send_pose(float x, float y, float rot,float confidence)
{
   float data[4] = {x,y,rot,confidence};

     while(!send_bytes((uint8_t*)data,16))
      {
//tryConnectToServer();
        delay(500);
      }
}
void send_map()
{
   ArcNode* refNodes = GetRefNodes();     
            points[0] = S_DATA;
            for(int i=0;i<N_POINTS;i++)
            {
              memcpy(&points[i*8+4], &refNodes[i].point , 8);              
            }
        
            while(!send_bytes(points,8*N_POINTS+4))
            {
  //tryConnectToServer();
              delay(500);
            }
            printf("refNodes sent");
          
            GetNewNodes(newNodes);
            for(int i=0;i<N_POINTS;i++)
            {
              memcpy(&points[i*8+4], &newNodes[i].point , 8);      
            }
              
           
            while(!send_bytes(points,8*N_POINTS+4))
            {
  //tryConnectToServer();
              delay(500);
            }
              

            printf("newNodes sent");

            map_nodes(newNodes,(Point2D*)(points+4),GetPos(),GetYaw(),true);
            
              
            while(!send_bytes(points,8*N_POINTS+4))
            {
  //tryConnectToServer();
              delay(500);
            }
}
void resend_message()
{   
    send_bytes(send_buffer,1);
} 

Point2D current_pos;
float current_yaw;
int attempts = 0;
unsigned long time_waited=0;
void loop() { 
  connectToWiFi();
  //tryConnectToServer();
  
  unsigned long mills = millis();
  unsigned long dt = mills - prev_millis;
  prev_millis = mills;
  unsigned char receive_buffer[256];
  switch (sarah_state)
  {    
    case BOOT:     
      if(send_message(S_INIT))
      {    
        printf("> INIT SENT");   
        next_state = INITIALIZING;
        sarah_state = WAIT_ACK;    
      }     
      else
      {
        printf("> COULD NOT CONNECT TO HIVE MIND."); 
        printf("> RE-TRYING");     
        delay(1000);
      } 
      break;
    case WAIT_ACK:
    {
    int nBytes =get_bytes(receive_buffer,4);
      if(nBytes > 0)
      {
        time_waited = 0;
        if(receive_buffer[0] == R_ACK )
        { 
          printf("ACK RECEIVED");
          sarah_state = next_state;
        }
        else if(receive_buffer[0] == R_NACK  )
        {
          printf("NACK RECEIVED");
          resend_message();
        }  
        else 
        {
          printf("MESSAGE NOT UNDERSTOOD:%i",receive_buffer[0] );
          resend_message();
        }      
      }
      else if ( time_waited > 2000)
      {
        time_waited = 0;
       // printf("TIMEOUT");
        resend_message();
        if(attempts++ > 3)
        {
            printf("RESETTING");
            tryConnectToServer();
            sarah_state = BOOT;
            attempts=0;
        }
      }
      else
      {
        time_waited += dt;
      }
    break;
    }
  case INITIALIZING:  
      
      if(send_message(S_PREP))
      { 
        Reset(); 
        next_state = PREPARING;
        sarah_state = WAIT_ACK;
        printf("> PREP SENT");  
      } 
          
    break;
  case PREPARING:
      if(process_lidar_feed())
      {
        printf("> PROCESSING LIDAR FEED...");  
        if(TryMakeRefScan())
        {
          if(send_message(S_READY))
          {
            next_state = READY;
            sarah_state = WAIT_ACK;
            printf("> READY SENT");  
          }
        }      
      }
      break;
  case READY:
     if(send_message(S_GET_CMD))
      { 
        next_state = BEGIN_MOVE;
        sarah_state = WAIT_ACK;
        printf("> S_GET_CMD SENT");  
      } 
     
      break;
  case BEGIN_MOVE:
    {
      if(get_bytes(receive_buffer,12))
      {                
          printf("> X Y ROT RECEIVED");  
          memcpy(&received_x, receive_buffer , sizeof(float));
          memcpy(&received_y, receive_buffer + 4 , sizeof(float));
          memcpy(&received_rot, receive_buffer + 8 , sizeof(float));
          next_state = MOVING;
          sarah_state = MOVING;         
      }
      else
      {
          printf("> SOMETHING BAD");  
      }
      break;
    }
  case MOVING:
    {
      process_lidar_feed();
      if(Serial.available())
      { 
        printf("> MOVE LIDAR");  
        char c = Serial.read();//any key stroke will do
        if(c)
        { 
          next_state = ESTIMATING;
          sarah_state = ESTIMATING;
        }
      }
      break;
    }
  case ESTIMATING:
    process_lidar_feed();
    Transform t ;
    float confidence;
    if(received_rot != 0)
    {
      t = EstimateRotation(received_x,received_y,received_rot,confidence);

    }
    else
    {
      t= EstimateTranslation(received_x,received_y,received_rot,confidence);
    }
    current_pos = add(current_pos,t.Point);
    
		printf("X:%.2f, Y:%.2f, confidence:%.6f",current_pos.X,current_pos.Y,confidence);
    send_map();
    send_pose(t.Point.X,t.Point.Y,t.Angle,confidence);
    TryMakeRefScan();
    if(send_message(S_READY))
    {
      next_state = READY;
      sarah_state = WAIT_ACK;
      printf("> READY SENT");  
    }
  default:
    break;
  }
   
    // }
    //   break;
    // default:
    //   break;
    // }
    
  
  
}    


