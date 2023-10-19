/*
WE MARS
2023 07 24
  
\Core 0 code


*/


#ifndef CORE_ZERO_H
#define CORE_ZERO_H 1


#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg;               // CAN Config
CAN_frame_t rx_frame;
CAN_frame_t tx_frame;
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

TaskHandle_t Core_Zero;

extern char strGPS[200];
extern char strIMU[200];

const int CR0_ciCANTimer =  10000;


uint32_t CR0_u32Now;  //for timing testing
uint32_t CR0_u32Last;


unsigned long CR0_ulPreviousMicrosCore0;
unsigned long CR0_ulCurrentMicrosCore0;


unsigned int CR0_uiRx_EStop = 0;
unsigned int CR0_uiTxSequenceBuffer[11];
unsigned int CR0_uiTxSequenceIndex = 0;
unsigned int CR0_uiTxIndex = 0;
unsigned int CR0_uiTxPacketIndex = 0;
unsigned int CR0_uiTxPacketCount = 0;

unsigned int uiTxID = 0;

char strCAN_TxGPS[200];
char strCAN_TxIMU[200];

float fTempByteCount = 0.0;

extern unsigned int Get_GPS_Data_Size();
void Core_ZeroCode( void * pvParameters );
unsigned int LoadTxBuffer(unsigned int uiId, unsigned int uiPacketCount,unsigned  int uiPacketIndx);

void Core_ZEROInit()
{
   xTaskCreatePinnedToCore(
                    Core_ZeroCode,   /* Task function. */
                    "Core_Zero",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Core_Zero,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
};


void Core_ZeroCode( void * pvParameters )
{
  Serial.print("Core - ");
  Serial.print(xPortGetCoreID());
  Serial.println("   running ");


  //Core 0 Setup
  //-------------------------------------------------------------------------------------------
  CAN_cfg.speed = CAN_SPEED_125KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_21;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
   
   
  //loop function for core 0
  //-------------------------------------------------------------------------------------------
  for(;;)
  {
       unsigned long currentMillis = millis();

      // Receive next CAN frame from queue
      if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
       {

        
        switch(rx_frame.MsgID)
        {
          case 0:
          {
            
            CR0_uiRx_EStop = 0;
            CR0_uiTxSequenceIndex = 0;
            CR0_uiTxIndex = 0;
            CR0_uiTxSequenceBuffer[CR0_uiTxSequenceIndex] = 0;
            Serial.println("RX ZERO");
            break;
          }
          case 100://requesting GPS data
          {
            CR0_uiRx_EStop = 1;
            CR0_uiTxSequenceIndex += 1;
            CR0_uiTxSequenceBuffer[CR0_uiTxSequenceIndex] = 100;
            
            if(CR0_uiTxSequenceIndex == 10)
            {
              CR0_uiTxSequenceIndex = 1;
                                                    
            }
            if(CR0_uiTxSequenceIndex == CR0_uiTxIndex)
            {
              CR0_uiRx_EStop = 10;
            }
         
            Serial.println("RX GPS");
            
          break;
          }
          case 101:  //requesting IMU data 
          {
            CR0_uiRx_EStop = 1;
            CR0_uiTxSequenceIndex += 1;
            CR0_uiTxSequenceBuffer[CR0_uiTxSequenceIndex] = 101;
            
            if(CR0_uiTxSequenceIndex == 10)
            {
              CR0_uiTxSequenceIndex = 1;
                                                    
            }
            if(CR0_uiTxSequenceIndex == CR0_uiTxIndex)
            {
              CR0_uiRx_EStop = 10;
            }
         
            Serial.println("RX IMU");
            
          break;
          }
        }
      }
      
      // Send CAN Message
      CR0_ulCurrentMicrosCore0 = micros();
      if ((CR0_ulCurrentMicrosCore0 - CR0_ulPreviousMicrosCore0) >= CR0_ciCANTimer)
      {
        CR0_ulPreviousMicrosCore0 = CR0_ulCurrentMicrosCore0;
       if(CR0_uiRx_EStop) // no e stop
       {
         if(CR0_uiTxIndex != CR0_uiTxSequenceIndex) // CAN Sequence to send, therefore something to send
         {
           if(CR0_uiTxPacketIndex == 0)  //start of Tx packets to send, load packet lenght
           {
             CR0_uiTxPacketIndex = 1;
             switch(CR0_uiTxSequenceBuffer[CR0_uiTxIndex])
             {
               case 0:  //e stop
               {
                 CR0_uiTxPacketCount = 0;
                 break;
               }
               case 100://requesting GPS data
              {
                CR0_uiTxPacketCount = Get_GPS_Data_Size();
                if(CR0_uiTxPacketCount == 0)
                {
                  
                  uiTxID = 195; //GPS data invalid
                  CR0_uiTxPacketIndex = LoadTxBuffer(uiTxID, CR0_uiTxPacketCount,CR0_uiTxPacketIndex);
                  ESP32Can.CANWriteFrame(&tx_frame);

                }
                else
                {
                  fTempByteCount = CR0_uiTxPacketCount + 1;
                  CR0_uiTxPacketCount = ceil(fTempByteCount/8); //take number of bytes to send add one for packet size then divide byte count by max number of bytes to send
                  uiTxID = 110;//start of GPS data packets,  the first byte = number of packets 
                  CR0_uiTxPacketIndex = LoadTxBuffer(uiTxID, CR0_uiTxPacketCount,CR0_uiTxPacketIndex);
                  ESP32Can.CANWriteFrame(&tx_frame);
                }
                
                break;
              }
              case 101:  //requesting IMU data 
              {
                CR0_uiTxPacketCount = Get_GPS_Data_Size();
                if(CR0_uiTxPacketCount == 0)
                {
                  
                  uiTxID = 196; //IMU data invalid
                  CR0_uiTxPacketIndex = LoadTxBuffer(uiTxID, CR0_uiTxPacketCount,CR0_uiTxPacketIndex);
                  ESP32Can.CANWriteFrame(&tx_frame);

                }
                else
                {
                  fTempByteCount = CR0_uiTxPacketCount + 1;
                  CR0_uiTxPacketCount = ceil(fTempByteCount/8); //take number of bytes to send add one for packet size then divide byte count by max number of bytes to send
                  uiTxID = 150;//start of I<U data packets,  the first byte = number of packets 
                  CR0_uiTxPacketIndex = LoadTxBuffer(uiTxID, CR0_uiTxPacketCount,CR0_uiTxPacketIndex);
                  ESP32Can.CANWriteFrame(&tx_frame);
                }
              break;
              }
             }
            }
            else
            {
              
              CR0_uiTxPacketIndex = LoadTxBuffer(uiTxID, CR0_uiTxPacketCount,CR0_uiTxPacketIndex);
              ESP32Can.CANWriteFrame(&tx_frame);
              uiTxID += 1;
              if(CR0_uiTxPacketIndex == CR0_uiTxPacketCount)
              {
                  CR0_uiTxIndex += 1;
                  if(CR0_uiTxIndex > 10)
                  {
                    CR0_uiTxIndex = 0;
                  }
              }
            }
          }
          if(CR0_uiTxSequenceIndex == CR0_uiTxIndex)
          {
           CR0_uiTxSequenceIndex = 0;
           CR0_uiTxIndex = 0;
          }
        } 
      }
       
     
           
           
          
        
      
  }
}

unsigned int LoadTxBuffer(unsigned int uiId, unsigned int uiPacketCount,unsigned  int uiPacketIndx)
{
  unsigned int uiIndexThroughGPSSting;

  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = uiId;
  tx_frame.FIR.B.DLC = 8;
  if(uiPacketCount == 0)
  {
    tx_frame.FIR.B.DLC = 1;
    tx_frame.data.u8[0] = 0;
    return(0);
  }
  else
  {
    if((uiId >= 110) && (uiId <= 145))//GPS data
    {
      if(uiPacketIndx == 1)
      {
        tx_frame.data.u8[0] = uiPacketCount;
        for(uiIndexThroughGPSSting = 1;uiIndexThroughGPSSting < 8;uiIndexThroughGPSSting++)
        {
          tx_frame.data.u8[uiIndexThroughGPSSting] = strCAN_TxGPS[uiIndexThroughGPSSting];
        }
        return(uiPacketIndx + 1);
      }
      else
      {
        for(uiIndexThroughGPSSting = 0;uiIndexThroughGPSSting < 8;uiIndexThroughGPSSting++)
        {
          tx_frame.data.u8[uiIndexThroughGPSSting] = strCAN_TxGPS[uiIndexThroughGPSSting];
        }
        return(uiPacketIndx + 1);
      }
     
    }
    else if ((uiId >= 150) && (uiId <= 190))  //IMU data
    {
      if(uiPacketIndx == 1)
      {
        tx_frame.data.u8[0] = uiPacketCount;
        for(uiIndexThroughGPSSting = 1;uiIndexThroughGPSSting < 8;uiIndexThroughGPSSting++)
        {
          tx_frame.data.u8[uiIndexThroughGPSSting] = strCAN_TxIMU[uiIndexThroughGPSSting];
        }
        return(uiPacketIndx + 1);
      }
      else
      {
        for(uiIndexThroughGPSSting = 0;uiIndexThroughGPSSting < 8;uiIndexThroughGPSSting++)
        {
          tx_frame.data.u8[uiIndexThroughGPSSting] = strCAN_TxGPS[uiIndexThroughGPSSting];
        }
        return(uiPacketIndx + 1);
      }
    }
  }
  

}



#endif
