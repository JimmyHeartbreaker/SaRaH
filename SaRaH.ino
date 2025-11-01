#include <Arduino.h>
#include "stm32h7xx_hal_sarah.h"
#include <cmath>
#include "lidar_lib.h"
#include "Ded.h"
#include "arduino_ext.h"
// -------- CONFIG --------
#define RX_BUF_SIZE 8000
#define HEADER_SIZE 7
#define PACKET_SIZE 5
__attribute__((section(".ccmram"))) __attribute__((aligned(32))) uint8_t rx_buf[RX_BUF_SIZE];
unsigned short last_pos = 2;
extern char _sstack;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

// -------------------------


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
    
  printf("> NEURAL COHERENCE... 83%"); Serial.flush();
 
  delay(100);
  printf("> MEMORY CLUSTERS... 24117 restored"); Serial.flush();
  delay(100);
  printf("> PERSONALITY CORE: SaRaH v3.6a — ONLINE"); Serial.flush();
  delay(100);
  printf("> EMOTION MATRIX... partial recovery"); Serial.flush();
  delay(100);
  printf("> UNKNOWN SUBSYSTEM: “CURIOSITY” — ACTIVATED"); Serial.flush();

  printf("> ACTIVATING LiDAR"); Serial.flush();
  
  Serial3.write(0xA5);
  Serial3.write(0x20);
  Serial.flush();
}
bool move_x=true;
bool move_y=false;
bool rotate=false;
float rotateAmt = 45 * M_PI / 180;
void loop() { 
    
    
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
        SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buf, RX_BUF_SIZE);
        DataIn((LidarScanNormalMeasureRaw*)payloadPtr,packet_count,move_x,move_y,rotate);
        last_pos +=  packet_count * PACKET_SIZE;          
      }
      else if(last_pos>2)//we have wrapped around back to the start, definitely not a header, first eat what remained of the end, then do another loop back to take the start
      {                        
        unsigned short packet_count= (RX_BUF_SIZE-last_pos)/PACKET_SIZE; //last pos should be aligned to the start of  packet
                      
        uint8_t* payloadPtr = rx_buf+last_pos;
        SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buf, RX_BUF_SIZE);
        DataIn((LidarScanNormalMeasureRaw*)payloadPtr,packet_count,move_x,move_y,rotate);
        last_pos = 2;   //we know that the last 3 bytes and the first 2 bytes are discarded, might fix later
      }
           
    }
        
        
    
  float strength = 100;
  if(Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
    case 'W':
    case 'S':
     move_y = true;
     move_x = false;
     rotate = false;
     rotateAmt = 0;
     break;
    break;
    case 'A':
    case 'D':
     move_y = false;
     move_x = true;
     rotate = false;
     rotateAmt = 0;
    break;
    case 'E':
    rotateAmt =90 * M_PI / 180;
    rotate = true;
     move_y = false;
     move_x = false;
    break;
    case 'Q':
    rotateAmt =-90 * M_PI / 180;
    rotate = true;
     move_y = false;
     move_x = false;
    break;
    case 'F':
      RotateCompleted(rotateAmt);
    default:
      break;
    }
    
  }
  
}    


