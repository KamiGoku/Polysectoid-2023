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
	    double increaseLEFTamount = /*int32_t((1-turningrate)*double(*/double(not_pause) * ( double(relax) * (double(full_contraction) * double( double(worm_pattern_turning[iteration][i]) +1)/2));//+1 means turning left
	    DEBUG_SERIAL.print("  Increase Amount: ");
	    // DEBUG_SERIAL.println(increase_amount);

	    int32_t currentILEFTposition = calibration[4*i] - int32_t((1-turningrate)*double(increaseLEFTamount));//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i], currentILEFTposition); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
	    int32_t trueLeftPosition = (int32_t)dxl.getCurAngle(DXL_ID[4*i]);//for debugging
	    DEBUG_SERIAL.println(trueLeftPosition - calibration[4*i]);
	    delay(30);

      double increaseRIGHTamount = /*int32_t((1+turningrate)*double(*/double(not_pause) * ( double(relax) * (double(full_contraction) * double( double(worm_pattern_turning[iteration][i]) -1)/2));//-1 means turning right

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
  delay(2200);//150);
}  

void peristalsis3DRoutine (DynamixelShield &dxl, int8_t worm_3D_pattern_peristalsis[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate){

  for(int i = 0;i<number_Of_Motor/4;i++){
	    double increase_amount =  double(not_pause) * (30 + double(full_contraction) * double(worm_3D_pattern_peristalsis[iteration][i])); //add 30 offset to remove slack cables
	    // DEBUG_SERIAL.print("  Increase Amount: ");
	    // DEBUG_SERIAL.println(increase_amount);

	    int32_t current4THposition = calibration[4*i+3] - int32_t(increase_amount);
	    dxl.setGoalAngle(DXL_ID[4*i+3], current4THposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Right Position(raw) : ");
	    int32_t true4THPosition = int32_t(dxl.getCurAngle(DXL_ID[4*i+3]));
	    // DEBUG_SERIAL.println(trueRightPosition - calibration[2*i+1]);
	    delay(20);

      int32_t current3RDposition = calibration[4*i+2] + int32_t(increase_amount);
	    dxl.setGoalAngle(DXL_ID[4*i+2], current3RDposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Right Position(raw) : ");
	    int32_t true3RDPosition = int32_t(dxl.getCurAngle(DXL_ID[4*i+2]));
	    // DEBUG_SERIAL.println(trueRightPosition - calibration[2*i+1]);
	    delay(20);

      int32_t current2NDposition = calibration[4*i+1] + int32_t(increase_amount);
	    dxl.setGoalAngle(DXL_ID[4*i+1], current2NDposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
	    int32_t true2NDPosition = int32_t(dxl.getCurAngle(DXL_ID[4*i+1]));
	    // DEBUG_SERIAL.println(trueLeftPosition - calibration[2*i]);
	    delay(20);

      int32_t current1STposition = calibration[4*i] - int32_t(increase_amount);
	    dxl.setGoalAngle(DXL_ID[4*i], current1STposition); //, UNIT_DEGREE);
	    // DEBUG_SERIAL.print(".   Present Left Position(raw) : ");
	    int32_t true1STPosition = int32_t(dxl.getCurAngle(DXL_ID[4*i]));
	    // DEBUG_SERIAL.println(trueLeftPosition - calibration[2*i]);
	    delay(20);


	    // DEBUG_SERIAL.print("  Segment Number: ");
	    // DEBUG_SERIAL.println(i+1);

      //delay(800);
	}
   delay(1000);
}

void undulation_3D_Obstacle (DynamixelShield &dxl, int8_t worm_3D_pattern_lateral[][SEGMENT_NUMBER], int8_t worm_3D_pattern_vertical[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration1, int iteration2,/* double vertical_deform,double lateral_deform,*/ int32_t full_contraction, bool not_pause, double turningrate){

  double slope_factor = 0.4;//How much of the front part of the worm lift its head up
  double alpha = log(2/min(max(slope_factor,0.01),1))/double(-1 + number_Of_Motor/4);
  for(int i = 0;i<number_Of_Motor/4;i++){
      //int32_t relax = int32_t(bool(worm_pattern_3D_turning[iteration][i]));   
      
      int8_t direction_lateral = worm_3D_pattern_lateral[iteration1][i];
      int8_t direction_vertical = worm_3D_pattern_vertical[iteration2][i];
      
      double vertical_deform = min(max(slope_factor*exp(alpha*double(i))-1,0),1);//expoentional function of weight variants from the tail to the head//double(i)/double(-1 + number_Of_Motor/4);
      double lateral_deform = 1 - vertical_deform;

	    double increase_1_amount = double(not_pause) * ( (double(full_contraction) * ( double(direction_vertical == 2) * vertical_deform + double (direction_lateral == 1)*lateral_deform )));//1 means turning in the 1_2 direction//make a change here?
	    DEBUG_SERIAL.print("  Increase Amount 1: ");
	    int32_t currentI_1_position = calibration[4*i] - int32_t(increase_1_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i], currentI_1_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 1 Position(raw) : ");
	    int32_t true1Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i]);//for debugging
	    DEBUG_SERIAL.println(true1Position - calibration[4*i]);
	    delay(20);

      double increase_2_amount = double(not_pause) * (  (double(full_contraction) * ( double(direction_lateral == 3)*lateral_deform + double(direction_vertical == 2)*vertical_deform )));//2 means turning in the 2_3 direction
	    DEBUG_SERIAL.print("  Increase Amount 2: ");
	    int32_t currentI_2_position = calibration[4*i+1] + int32_t(increase_2_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+1], currentI_2_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 2 Position(raw) : ");
	    int32_t true2Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+1]);//for debugging
	    DEBUG_SERIAL.println(true2Position - calibration[4*i+1]);
	    delay(20);

      double increase_3_amount = double(not_pause) * ( (double(full_contraction) * ( double(direction_vertical == 4)*vertical_deform + double(direction_lateral == 3)*lateral_deform )));//3 means turning in the 3_4 direction
	    DEBUG_SERIAL.print("  Increase Amount 3: ");
	    int32_t currentI_3_position = calibration[4*i+2] + int32_t(increase_3_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+2], currentI_3_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 3 Position(raw) : ");
	    int32_t true3Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+2]);//for debugging
	    DEBUG_SERIAL.println(true3Position - calibration[4*i+2]);
	    delay(20);

      double increase_4_amount = double(not_pause) * ( (double(full_contraction) * (double (direction_lateral == 1)*lateral_deform + double(direction_vertical == 4)*vertical_deform )));//4 means turning in the 4_1 direction
	    DEBUG_SERIAL.print("  Increase Amount 4: ");
	    int32_t currentI_4_position = calibration[4*i+3] - int32_t(increase_4_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+3], currentI_4_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 4 Position(raw) : ");
	    int32_t true4Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+3]);//for debugging
	    DEBUG_SERIAL.println(true4Position - calibration[4*i+3]);
	    delay(20);

	    DEBUG_SERIAL.print("  Segment Number: ");
	    DEBUG_SERIAL.println(i);
  }
  delay(100);//150);
} 

double direction_Mapping_Function (int8_t motor, int8_t lateral, int8_t vertical, double lateral_deform, double vertical_deform){
  float B1 = float(lateral==1);
  float B2 = float(vertical==2);
  float B3 = float(lateral==3);
  float B4 = float(vertical==4);
  float contraction = 0;
  if(motor == 1){
    contraction = max(min(0.5*pow((B1-B3)*float(lateral_deform),3) + 0.5 +0.1*abs(0.5*pow((B2+B4)*float(vertical_deform),3)+0.5) ,1),0);
    return double(contraction);
  }
  if(motor == 3){
    contraction = max(min(0.5*pow((B3-B1)*float(lateral_deform),3) + 0.5 +0.1*abs(0.5*pow((B2+B4)*float(vertical_deform),3)+0.5) ,1),0);
    return double(contraction);
  }
   if(motor == 2){
    contraction = max(min(0.5*pow((B2-B4)*float(vertical_deform),3)+0.5 +0.1*abs(0.5*pow((B1+B3)*float(lateral_deform),3)+0.5) ,1),0);
    return double(contraction);
  }
   if(motor == 4){
    contraction = max(min(0.5*pow((B4-B2)*float(vertical_deform),3)+0.5 +0.1*abs(0.5*pow((B1+B3)*float(lateral_deform),3)+0.5) ,1),0);
    return double(contraction);
  }
  return(contraction);
}


void undulation_3D_Obstacle_Reoriented (DynamixelShield &dxl, int8_t worm_3D_pattern_lateral[][SEGMENT_NUMBER], int8_t worm_3D_pattern_vertical[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration1, int iteration2,/* double vertical_deform,double lateral_deform,*/ int32_t full_contraction, bool not_pause, double turningrate){

  double slope_factor = 1;//How much of the front part of the worm lift its head up
  double alpha = log(2/min(max(slope_factor,0.01),1))/double(-1 + number_Of_Motor/4);
  for(int i = 0;i<number_Of_Motor/4;i++){
      //int32_t relax = int32_t(bool(worm_pattern_3D_turning[iteration][i]));   
      
      int8_t direction_lateral = worm_3D_pattern_lateral[iteration1][i];
      int8_t direction_vertical = worm_3D_pattern_vertical[iteration2][i];
      
      double vertical_deform = min(max(slope_factor*exp(alpha*double(i))-1,0),1);//expoentional function of weight variants from the tail to the head//double(i)/double(-1 + number_Of_Motor/4);
      double lateral_deform = sqrt(1 - sq(vertical_deform));

	    double increase_1_amount = double(not_pause) * ( (double(full_contraction) * (direction_Mapping_Function(1, direction_lateral, direction_vertical, lateral_deform, vertical_deform))));//1 means turning in the 1_2 direction//make a change here?
	    DEBUG_SERIAL.print("  Increase Amount 1: ");
	    int32_t currentI_1_position = calibration[4*i] - int32_t(increase_1_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i], currentI_1_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 1 Position(raw) : ");
	    int32_t true1Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i]);//for debugging
	    DEBUG_SERIAL.println(true1Position - calibration[4*i]);
	    delay(20);

      double increase_2_amount = double(not_pause) * (  (double(full_contraction) * (direction_Mapping_Function(2, direction_lateral, direction_vertical, lateral_deform, vertical_deform))));//2 means turning in the 2_3 direction
	    DEBUG_SERIAL.print("  Increase Amount 2: ");
	    int32_t currentI_2_position = calibration[4*i+1] + int32_t(increase_2_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+1], currentI_2_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 2 Position(raw) : ");
	    int32_t true2Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+1]);//for debugging
	    DEBUG_SERIAL.println(true2Position - calibration[4*i+1]);
	    delay(20);

      double increase_3_amount = double(not_pause) * ( (double(full_contraction) * (direction_Mapping_Function(3, direction_lateral, direction_vertical, lateral_deform, vertical_deform))));//3 means turning in the 3_4 direction
	    DEBUG_SERIAL.print("  Increase Amount 3: ");
	    int32_t currentI_3_position = calibration[4*i+2] + int32_t(increase_3_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+2], currentI_3_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 3 Position(raw) : ");
	    int32_t true3Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+2]);//for debugging
	    DEBUG_SERIAL.println(true3Position - calibration[4*i+2]);
	    delay(20);

      double increase_4_amount = double(not_pause) * ( (double(full_contraction) * (direction_Mapping_Function(4, direction_lateral, direction_vertical, lateral_deform, vertical_deform))));//4 means turning in the 4_1 direction
	    DEBUG_SERIAL.print("  Increase Amount 4: ");
	    int32_t currentI_4_position = calibration[4*i+3] - int32_t(increase_4_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+3], currentI_4_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 4 Position(raw) : ");
	    int32_t true4Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+3]);//for debugging
	    DEBUG_SERIAL.println(true4Position - calibration[4*i+3]);
	    delay(20);

	    DEBUG_SERIAL.print("  Segment Number: ");
	    DEBUG_SERIAL.println(i);
  }
  delay(100);//150);
} 

double sum_Weight_Curve(double a, int i){
  return(a*(i^2));
}

void undulation_Headbob_3D_Obstacle (DynamixelShield &dxl, int8_t worm_3D_pattern_lateral[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration1, double headlift_Percentage,/* double vertical_deform,double lateral_deform,*/ int32_t full_contraction, bool not_pause, double turningrate){

  double slope_factor = 0.4;//How much of the front part of the worm lift its head up
  float alpha = (2/min(max(slope_factor,0.01),1))/double(-1 + number_Of_Motor/4);
  for(int i = 0;i<number_Of_Motor/4;i++){
      //int32_t relax = int32_t(bool(worm_pattern_3D_turning[iteration][i]));   
      
      int8_t direction_lateral = worm_3D_pattern_lateral[iteration1][i];
      //double direction_vertical = headlift_Percentage   //* worm_3D_pattern_vertical[iteration2][i];
      
      double vertical_deform = min(max(slope_factor*exp(alpha*double(i-2))-1,0),1);//expoentional function of weight variants from the tail to the head//double(i)/double(-1 + number_Of_Motor/4);
      double lateral_deform = 1 + sum_Weight_Curve(0.008333, i) - vertical_deform;

	    double increase_1_amount = double(not_pause) * ( (double(full_contraction) * ( 0 * vertical_deform + double (direction_lateral == 1)*lateral_deform )));//1 means turning in the 1_2 direction//make a change here?
	    DEBUG_SERIAL.print("  Increase Amount 1: ");
	    int32_t currentI_1_position = calibration[4*i] - int32_t(increase_1_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i], currentI_1_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 1 Position(raw) : ");
	    int32_t true1Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i]);//for debugging
	    DEBUG_SERIAL.println(true1Position - calibration[4*i]);
	    delay(20);

      double increase_2_amount = double(not_pause) * (  (double(full_contraction) * ( double(direction_lateral == 3)*lateral_deform + 0*vertical_deform )));//2 means turning in the 2_3 direction
	    DEBUG_SERIAL.print("  Increase Amount 2: ");
	    int32_t currentI_2_position = calibration[4*i+1] + int32_t(increase_2_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+1], currentI_2_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 2 Position(raw) : ");
	    int32_t true2Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+1]);//for debugging
	    DEBUG_SERIAL.println(true2Position - calibration[4*i+1]);
	    delay(20);

      double increase_3_amount = double(not_pause) * ( (double(full_contraction) * ( headlift_Percentage*vertical_deform + double(direction_lateral == 3)*lateral_deform )));//3 means turning in the 3_4 direction
	    DEBUG_SERIAL.print("  Increase Amount 3: ");
	    int32_t currentI_3_position = calibration[4*i+2] + int32_t(increase_3_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+2], currentI_3_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 3 Position(raw) : ");
	    int32_t true3Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+2]);//for debugging
	    DEBUG_SERIAL.println(true3Position - calibration[4*i+2]);
	    delay(20);

      double increase_4_amount = double(not_pause) * ( (double(full_contraction) * (double (direction_lateral == 1)*lateral_deform + headlift_Percentage*vertical_deform )));//4 means turning in the 4_1 direction
	    DEBUG_SERIAL.print("  Increase Amount 4: ");
	    int32_t currentI_4_position = calibration[4*i+3] - int32_t(increase_4_amount);//actual update with calibration data
	    dxl.setGoalAngle(DXL_ID[4*i+3], currentI_4_position); //, UNIT_DEGREE);
	    DEBUG_SERIAL.print(".   Present 4 Position(raw) : ");
	    int32_t true4Position = (int32_t)dxl.getCurAngle(DXL_ID[4*i+3]);//for debugging
	    DEBUG_SERIAL.println(true4Position - calibration[4*i+3]);
	    delay(20);

	    DEBUG_SERIAL.print("  Segment Number: ");
	    DEBUG_SERIAL.println(i);
  }
  delay(100);//150);
} 