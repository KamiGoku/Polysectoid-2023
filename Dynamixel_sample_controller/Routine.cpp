#include <arduino.h>
#include <DynamixelShield.h>
#define SEGMENT_NUMBER 7

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

void peristalsisRoutine (DynamixelShield &dxl, int8_t worm_pattern[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, float turningrate){

  for(int i = 0;i<number_Of_Motor/2;i++){
	    int32_t increase_amount =  int32_t(not_pause) * (50 + full_contraction * int32_t(worm_pattern[iteration][i])); //add 30 offset to remove slack cables
	    // DEBUG_SERIAL.print("  Increase Amount: ");
	    // DEBUG_SERIAL.println(increase_amount);

	    int32_t currentIRIGHTposition = calibration[2*i+1] + int32_t((1-turningrate)*double(increase_amount));
	    dxl.setGoalAngle(DXL_ID[2*i+1], currentIRIGHTposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Right Position(raw) : ");
	    int32_t trueRightPosition = int32_t(dxl.getCurAngle(DXL_ID[2*i+1]));
	    // DEBUG_SERIAL.println(trueRightPosition - calibration[2*i+1]);
	    delay(40);

      int32_t currentILEFTposition = calibration[2*i] - int32_t((1+turningrate)*double(increase_amount));
	    dxl.setGoalAngle(DXL_ID[2*i], currentILEFTposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
	    int32_t trueLeftPosition = int32_t(dxl.getCurAngle(DXL_ID[2*i]));
	    // DEBUG_SERIAL.println(trueLeftPosition - calibration[2*i]);
	    delay(40);

	    // DEBUG_SERIAL.print("  Segment Number: ");
	    // DEBUG_SERIAL.println(i+1);

      //delay(800);
	}
   //delay(200);
}

void undulationRoutine (DynamixelShield &dxl, int8_t worm_pattern_turning[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, float turningrate){

  for(int i = 0;i<number_Of_Motor/2;i++){
      int32_t relax = abs(worm_pattern_turning[iteration][i]);      
	    int32_t increaseLEFTamount = int32_t((1-turningrate)*double(int32_t(not_pause) * ( relax * (full_contraction * int32_t(worm_pattern_turning[iteration][i]+1)/2))));//+1 means turning left
	    DEBUG_SERIAL.print("  Increase Amount: ");
	    // DEBUG_SERIAL.println(increase_amount);

	    int32_t currentILEFTposition = calibration[2*i] - increaseLEFTamount;//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[2*i], currentILEFTposition); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
	    int32_t trueLeftPosition = (int32_t)dxl.getCurAngle(DXL_ID[2*i]);//for debugging
	    DEBUG_SERIAL.println(trueLeftPosition - calibration[2*i]);
	    delay(30);

      int32_t increaseRIGHTamount = int32_t((1+turningrate)*double(int32_t(not_pause) * ( relax * (full_contraction * int32_t(worm_pattern_turning[iteration][i]-1)/2))));//-1 means turning right

	    int32_t currentIRIGHTposition = calibration[2*i+1] - increaseRIGHTamount;//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[2*i+1], currentIRIGHTposition); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present Right Position(raw) : ");
	    int32_t trueRightPosition = (int32_t)dxl.getCurAngle(DXL_ID[2*i+1]);//for debugging
	    DEBUG_SERIAL.println(trueRightPosition - calibration[2*i+1]);
	    delay(30);

	    DEBUG_SERIAL.print("  Segment Number: ");
	    DEBUG_SERIAL.println(i);
  }
  // delay(70);
}  
