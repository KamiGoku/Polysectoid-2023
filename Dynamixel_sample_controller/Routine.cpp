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

void peristalsisRoutine (DynamixelShield &dxl, int8_t worm_pattern[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate){

  for(int i = 0;i<number_Of_Motor/4;i++){
	    double increase_amount =  double(not_pause) * (30 + double(full_contraction) * double(worm_pattern[iteration][i])); //add 30 offset to remove slack cables
	    // DEBUG_SERIAL.print("  Increase Amount: ");
	    // DEBUG_SERIAL.println(increase_amount);

	    int32_t currentIRIGHTposition = calibration[4*i+2] + int32_t((1-turningrate)*double(increase_amount));
	    dxl.setGoalAngle(DXL_ID[4*i+2], currentIRIGHTposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Right Position(raw) : ");
	    int32_t trueRightPosition = int32_t(dxl.getCurAngle(DXL_ID[4*i+2]));
	    // DEBUG_SERIAL.println(trueRightPosition - calibration[2*i+1]);
	    delay(40);

      int32_t currentILEFTposition = calibration[4*i] - int32_t((1+turningrate)*double(increase_amount));
	    dxl.setGoalAngle(DXL_ID[4*i], currentILEFTposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
	    int32_t trueLeftPosition = int32_t(dxl.getCurAngle(DXL_ID[4*i]));
	    // DEBUG_SERIAL.println(trueLeftPosition - calibration[2*i]);
	    delay(40);

	    // DEBUG_SERIAL.print("  Segment Number: ");
	    // DEBUG_SERIAL.println(i+1);

      //delay(800);
	}
   //delay(200);
}

void undulationRoutine (DynamixelShield &dxl, int8_t worm_pattern_turning[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate){

  for(int i = 0;i<number_Of_Motor/4;i++){
      int32_t relax = abs(worm_pattern_turning[iteration][i]);      
	    int32_t increaseLEFTamount = /*int32_t((1-turningrate)*double(*/int32_t(not_pause) * ( relax * (full_contraction * int32_t(worm_pattern_turning[iteration][i]+1)/2));//+1 means turning left
	    DEBUG_SERIAL.print("  Increase Amount: ");
	    // DEBUG_SERIAL.println(increase_amount);

	    int32_t currentILEFTposition = calibration[4*i] - int32_t((1-turningrate)*double(increaseLEFTamount));//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i], currentILEFTposition); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
	    int32_t trueLeftPosition = (int32_t)dxl.getCurAngle(DXL_ID[4*i]);//for debugging
	    DEBUG_SERIAL.println(trueLeftPosition - calibration[4*i]);
	    delay(30);

      int32_t increaseRIGHTamount = /*int32_t((1+turningrate)*double(*/int32_t(not_pause) * ( relax * (full_contraction * int32_t(worm_pattern_turning[iteration][i]-1)/2));//-1 means turning right

	    int32_t currentIRIGHTposition = calibration[4*i+2] - int32_t((1+turningrate)*double(increaseRIGHTamount));//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+2], currentIRIGHTposition); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present Right Position(raw) : ");
	    int32_t trueRightPosition = (int32_t)dxl.getCurAngle(DXL_ID[4*i+2]);//for debugging
	    DEBUG_SERIAL.println(trueRightPosition - calibration[4*i+2]);
	    delay(30);

	    DEBUG_SERIAL.print("  Segment Number: ");
	    DEBUG_SERIAL.println(i);
  }
  // delay(70);
}  

void turning3DRoutine (DynamixelShield &dxl, int8_t worm_pattern_3D_turning[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate){

  for(int i = 0;i<number_Of_Motor/4;i++){
      int32_t relax = int32_t(bool(worm_pattern_3D_turning[iteration][i]));   
      int8_t direction = worm_pattern_3D_turning[iteration][i];

	    int32_t increase_1_amount = int32_t(not_pause) * ( relax * (full_contraction * int32_t( (direction == 1)||(direction == 2) )));//1 means turning in the 1_2 direction
	    DEBUG_SERIAL.print("  Increase Amount 1: ");
	    int32_t currentI_1_position = calibration[4*i] - int32_t(/*(1-turningrate)**/double(increase_1_amount));//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i], currentI_1_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 1 Position(raw) : ");
	    int32_t true1Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i]);//for debugging
	    DEBUG_SERIAL.println(true1Position - calibration[4*i]);
	    delay(20);

      int32_t increase_2_amount = int32_t(not_pause) * ( relax * (full_contraction * int32_t( (direction == 2)||(direction == 3) )));//2 means turning in the 2_3 direction
	    DEBUG_SERIAL.print("  Increase Amount 2: ");
	    int32_t currentI_2_position = calibration[4*i+1] + int32_t(/*(1-turningrate)**/double(increase_2_amount));//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+1], currentI_2_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 2 Position(raw) : ");
	    int32_t true2Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+1]);//for debugging
	    DEBUG_SERIAL.println(true2Position - calibration[4*i+1]);
	    delay(20);

      int32_t increase_3_amount = int32_t(not_pause) * ( relax * (full_contraction * int32_t( (direction == 3)||(direction == 4) )));//3 means turning in the 3_4 direction
	    DEBUG_SERIAL.print("  Increase Amount 3: ");
	    int32_t currentI_3_position = calibration[4*i+2] + int32_t(/*(1-turningrate)**/double(increase_3_amount));//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+2], currentI_3_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 3 Position(raw) : ");
	    int32_t true3Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+2]);//for debugging
	    DEBUG_SERIAL.println(true3Position - calibration[4*i+2]);
	    delay(20);

      int32_t increase_4_amount = int32_t(not_pause) * ( relax * (full_contraction * int32_t( (direction == 4)||(direction == 1) )));//4 means turning in the 4_1 direction
	    DEBUG_SERIAL.print("  Increase Amount 4: ");
	    int32_t currentI_4_position = calibration[4*i+3] - int32_t(/*(1-turningrate)**/double(increase_4_amount));//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+3], currentI_4_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 4 Position(raw) : ");
	    int32_t true4Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+3]);//for debugging
	    DEBUG_SERIAL.println(true4Position - calibration[4*i+3]);
	    delay(20);

	    DEBUG_SERIAL.print("  Segment Number: ");
	    DEBUG_SERIAL.println(i);
  }
  delay(300);//150);
}  
