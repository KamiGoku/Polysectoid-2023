/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID[6] = {1,2,3,4,5,6};
const int number_Of_Motor = sizeof(DXL_ID) / sizeof(DXL_ID[0]);
const float DXL_PROTOCOL_VERSION = 2.0;

const int32_t worm_pattern[3][3] = {{1,0,1},{0,1,1},{1,1,0}};
int32_t iteration = 0;
int32_t calibration[6];

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  for(int i = 0;i<number_Of_Motor;i++){ //iterate through all the motors
    dxl.ping(DXL_ID[i]);
    delay(800);
  
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_EXTENDED_POSITION);
    delay(800);
    dxl.torqueOn(DXL_ID[i]);
    delay(800);
    calibration[i] = (int32_t)dxl.getCurAngle(DXL_ID[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  for(int i = 0;i<number_Of_Motor/2;i++){
    int32_t increase_amount = 100 * (int32_t)worm_pattern[iteration][i];
    DEBUG_SERIAL.print("  Increase Amount: ");
    DEBUG_SERIAL.println(increase_amount);
    int32_t currentILEFTposition = calibration[2*i] - increase_amount;
    dxl.setGoalAngle(DXL_ID[2*i], currentILEFTposition); //, UNIT_DEGREE);
    DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
    int32_t trueLeftPosition = (int32_t)dxl.getCurAngle(DXL_ID[2*i]);
    DEBUG_SERIAL.println(trueLeftPosition - calibration[2*i]);

    int32_t currentIRIGHTposition = calibration[2*i+1] + increase_amount;
    dxl.setGoalAngle(DXL_ID[2*i+1], currentIRIGHTposition); //, UNIT_DEGREE);
    DEBUG_SERIAL.print(".   Present Right Position(raw) : ");
    int32_t trueRightPosition = (int32_t)dxl.getCurAngle(DXL_ID[2*i+1]);
    DEBUG_SERIAL.println(trueRightPosition - calibration[2*i+1]);
    DEBUG_SERIAL.print("  Segment Number: ");
    DEBUG_SERIAL.println(i);
  }
    
    // Print present position in raw value
    // DEBUG_SERIAL.print("Present Position(raw) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i]));
    // DEBUG_SERIAL.print("Present Position(raw) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i+1]));
    // delay(1000);
  
    // Set Goal Position in DEGREE value
    // dxl.setGoalPosition(DXL_ID[2*i], currentILEFTposition, UNIT_DEGREE);
    // dxl.setGoalPosition(DXL_ID[2*i+1], currentIRIGHTposition, UNIT_DEGREE);
    // delay(1000);
    // Print present position in degree value
    // DEBUG_SERIAL.print("Present Position(degree) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i], UNIT_DEGREE));
    // DEBUG_SERIAL.print("Present Position(degree) : ");
    // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2*i+1], UNIT_DEGREE));
    // delay(1000);

  iteration++;
  iteration = iteration % 3;
  DEBUG_SERIAL.print("Iteration Number: ");
  DEBUG_SERIAL.println(iteration);
  delay(1500);

}
