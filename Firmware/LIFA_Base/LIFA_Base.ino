


/*********************************************************************************
 **
 **  LVFA_Firmware - Provides Basic Arduino Sketch For Interfacing With LabVIEW.
 **
 **  Written By:    Sam Kristoff - National Instruments
 **  Written On:    November 2010
 **  Last Updated:  Dec 2011 - Kevin Fort - National Instruments
 **
 **  This File May Be Modified And Re-Distributed Freely. Original File Content
 **  Written By Sam Kristoff And Available At www.ni.com/arduino.
 **
 *********************************************************************************/


/*********************************************************************************
 **
 ** Includes.
 **
 ********************************************************************************/ 

// Standard includes.  These should always be included.
#include <Wire.h>    //是I2C通讯的，Arduino和I2C设备通讯使用的(SCLK DIN RCLK)
#include <SPI.h>    //SPI通讯
#include <Servo.h> //控制舵机
#include "LabVIEWInterface.h"
#include "U8glib.h" //12864液晶
#include "u8g.h"   //12864液晶
#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A

#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values. 将满量程范围设置为+/- 2,4或8g。



/*********************************************************************************
 **  setup()
 **
 **  Initialize the Arduino and setup serial communication.
 **
 **  Input:  None
 **  Output: None
 *********************************************************************************/
void setup()
{  
  // Initialize Serial Port With The Default Baud Rate
  syncLV();
  // Place your custom setup code here
  analogWrite(5,0);
  analogWrite(6,0);

}
void(* resetFunc) (void) = 0;//软复位函数
/*********************************************************************************
 **  loop()
 **
 **  The main loop.  This loop runs continuously on the Arduino.  It 
 **  receives and processes serial commands from LabVIEW.
 **
 **  Input:  None
 **  Output: None
 *********************************************************************************/
void loop()
{   
  // Check for commands from LabVIEW and process them.   
 
  checkForCommand();
  // Place your custom loop code here (this may slow down communication with LabVIEW)
  
  
  if(acqMode==1)
  {
    sampleContinously();
  }

}














