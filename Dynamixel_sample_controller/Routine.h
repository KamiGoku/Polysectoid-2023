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

void peristalsisRoutine (DynamixelShield &dxl, int8_t worm_pattern[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate);
void undulationRoutine (DynamixelShield &dxl, int8_t worm_pattern_turning[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate);
void turning3DRoutine (DynamixelShield &dxl, int8_t worm_pattern_3D_turning[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate);
void peristalsis3DRoutine (DynamixelShield &dxl, int8_t worm_3D_pattern_peristalsis[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration, int32_t full_contraction, bool not_pause, double turningrate);
void undulation_3D_Obstacle (DynamixelShield &dxl, int8_t worm_3D_pattern_lateral[][SEGMENT_NUMBER], int8_t worm_3D_pattern_vertical[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration1, int iteration2,/* double vertical_deform, double lateral_deform, */int32_t full_contraction, bool not_pause, double turningrate);
void undulation_3D_Obstacle_Reoriented (DynamixelShield &dxl, int8_t worm_3D_pattern_lateral[][SEGMENT_NUMBER], int8_t worm_3D_pattern_vertical[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration1, int iteration2,/* double vertical_deform,double lateral_deform,*/ int32_t full_contraction, bool not_pause, double turningrate);
void undulation_Headbob_3D_Obstacle (DynamixelShield &dxl, int8_t worm_3D_pattern_lateral[][SEGMENT_NUMBER], int number_Of_Motor, int32_t calibration[], uint8_t DXL_ID[], int iteration1, double headlift_Percentage,/* double vertical_deform,double lateral_deform,*/ int32_t full_contraction, bool not_pause, double turningrate);
