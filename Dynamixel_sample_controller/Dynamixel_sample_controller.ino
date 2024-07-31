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
#include <SimpleCLI.h>
#include "Routine.h" 
#define SEGMENT_NUMBER 7
#define PERISTALSIS_CYCLES_NUMBER 0//2000 //1:40 min = 100,000 ms;100,000ms/(30ms*14) ~= 238
#define UNDULATION_CYCLES_NUMBER 0  //1:30 min = 90,000 ms;90,000ms/(40ms*14) ~= 161
#define TURNING_3D_CYCLES_NUMBER 0
#define PERISTALSIS_3D_CYCLES_NUMBER 0
#define UNDULATION_3D_OBSTACLE 2000

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

uint8_t DXL_ID[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28};
const int number_Of_Motor = sizeof(DXL_ID) / sizeof(DXL_ID[0]);
const float DXL_PROTOCOL_VERSION = 2.0;
double turningrate = 0;//0.15;//-0.24; 
int8_t worm_pattern[][SEGMENT_NUMBER] = { {0,0,1,1,0,0,1},
                                          {1,0,0,1,1,0,0},
                                          {1,1,0,0,1,1,0},
                                          {0,1,1,0,0,1,1}
                                          /*{0,1,1,0,0,1,1},
                                          {0,0,1,1,0,0,1},
                                          {1,0,0,1,1,0,0},
                                          {1,1,0,0,1,1,0}/*,
                                          {1,1,0,0,1,1,1}*/}; //1 means contract, 0 means relax
                                          
int8_t worm_pattern_turning[][SEGMENT_NUMBER] = { /*{1,1,1,0,-1,-1,-1},
                                                  {0,1,1,1,0,-1,-1},
                                                  {-1,0,1,1,1,0,-1}, 
                                                  {-1,-1,0,1,1,1,0},
                                                  {-1,-1,-1,0,1,1,1},
                                                  {0,-1,-1,-1,0,1,1},
                                                  {1,0,-1,-1,-1,0,1},
                                                  {1, 1,0,-1,-1,-1,0}*/
                                                  //{1,1,1,1,1,1,1}
                                                  {1, 1,0,-1,-1,-1,0},
                                                  {1,0,-1,-1,-1,0,1},
                                                  {0,-1,-1,-1,0,1,1},
                                                  {-1,-1,-1,0,1,1,1},
                                                  {-1,-1,0,1,1,1,0},
                                                  {-1,0,1,1,1,0,-1}, 
                                                  {0,1,1,1,0,-1,-1},
                                                  {1,1,1,0,-1,-1,-1}}; //-1 means turning left. 1 means turning right, 0 means not turning
int8_t worm_pattern_3D_turning[][SEGMENT_NUMBER] = {/*{1,2,3,0,0,0,0},
                                                    {2,3,4,0,0,0,0},
                                                    {3,4,1,0,0,0,0},
                                                    {4,1,2,0,0,0,0},
                                                    {1,2,3,0,0,0,0},
                                                    {2,3,4,0,0,0,0},
                                                    {3,4,1,0,0,0,0},
                                                    {4,1,2,0,0,0,0},*/
                                                    
                                                    {1,1,1,0,0,0,0},
                                                    {2,2,2,0,0,0,0},
                                                    {3,3,3,0,0,0,0},
                                                    {4,4,4,0,0,0,0}
                                                    };
int8_t worm_3D_pattern_peristalsis[][SEGMENT_NUMBER]={{0,0,1,1,0,0,1},
                                                       {1,0,0,1,1,0,0},
                                                       {1,1,0,0,1,1,0},
                                                       {0,1,1,0,0,1,1}
                                                      }; 
  bool pause = true;
int8_t worm_3D_pattern_lateral[][SEGMENT_NUMBER]={{1,1,1,0,3,3,3},
                                                    {1,1,0,3,3,3,0},
                                                    {1,0,3,3,3,0,1},
                                                    {0,3,3,3,0,1,1},
                                                    {3,3,3,0,1,1,1},
                                                    {3,3,0,1,1,1,0},
                                                    {3,0,1,1,1,0,3},
                                                    {0,1,1,1,0,3,3},
                                                    };
int8_t worm_3D_pattern_vertical[][SEGMENT_NUMBER]={{2,2,2,2,0,4,4},
                                                  {2,2,2,0,4,4,0},
                                                  {2,2,0,4,4,0,0},
                                                  {2,0,4,4,0,0,2},
                                                  {0,4,4,0,0,2,2},
                                                  {4,4,0,0,2,2,2},
                                                  {4,0,0,2,2,2,2},
                                                  {0,0,2,2,2,2,0},
                                                  {0,2,2,2,2,0,4,},
                                                  };
const int pause_button = 1;


int32_t peristalsis_cycle_size = sizeof(worm_pattern) / sizeof(worm_pattern[0]);
int32_t undulation_cycle_size = sizeof(worm_pattern_turning) / sizeof(worm_pattern_turning[0]);
int32_t turning_3d_cycle_size = sizeof(worm_pattern_3D_turning) / sizeof(worm_pattern_3D_turning[0]);
int32_t peristalsis_3D_cycle_size = sizeof(worm_3D_pattern_peristalsis) / sizeof(worm_3D_pattern_peristalsis[0]);
int32_t obstacle_3D_cycle_lateral_size = sizeof(worm_3D_pattern_lateral) / sizeof(worm_3D_pattern_lateral[0]);
int32_t obstacle_3D_cycle_vertical_size = sizeof(worm_3D_pattern_vertical) / sizeof(worm_3D_pattern_vertical[0]);

int iteration = 0;//2_D motion state machine index tracking
int iteration1 = 0;//3_D lateral motion state machine index tracking
int iteration2 = 0;//3_D vertical motion state machine index tracking

int32_t calibration[number_Of_Motor]= {}; //{131, 251, 218, 172, 284, 165, 357, 198, 308, 132, 257, 226, 302, 40};//{162, 100, 24, 240, 136, 334, 127, 6, 355, 304, 226, 168, 2, 268};
const int32_t full_contraction_peristalsis = 700;//550;//700;//1000;//850;
const int32_t full_contraction_undulation = 850;//1300;
const int32_t full_contraction_3D_turn = 500;
const int32_t full_contraction_3D_peristalsis = 500;//300;
const int32_t full_contraction_3D_undulation_obstacle = 600;

//double vertical_deform=0.3;
double lateral_deform=0.7;

DynamixelShield dxl;

//Command line input

// Create CLI Object
SimpleCLI cli;

// Commands
Command run;
Command stop;

void pingCallback(cmd* c) {
    Command cmd(c); // Create wrapper object

    // Get arguments
    Argument numberArg = cmd.getArgument("number");
    Argument strArg    = cmd.getArgument("str");
    Argument cArg      = cmd.getArgument("c");

    // Get values
    int numberValue = numberArg.getValue().toInt();
    String strValue = strArg.getValue();
    bool   cValue   = cArg.isSet();

    if (cValue) strValue.toUpperCase();

    // Print response
    for (int i = 0; i<numberValue; i++) DEBUG_SERIAL.println(strValue);
}

    // Callback in case of an error
void errorCallback(cmd_error* e) {
    CommandError cmdError(e); // Create wrapper object

    DEBUG_SERIAL.print("ERROR: ");
    DEBUG_SERIAL.println(cmdError.toString());

    if (cmdError.hasCommand()) {
        DEBUG_SERIAL.print("Did you mean \"");
        DEBUG_SERIAL.print(cmdError.getCommand().toString());
        DEBUG_SERIAL.println("\"?");
    }
}

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  // pinMode(pause_button, INPUT);
  // attachInterrupt(digitalPinToInterrupt(pause_button), relax_worm, RISING); //pause when button is pressed and unpaused when it's pressed again
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  delay(1000);
  DEBUG_SERIAL.print("Callibrated segment set: {");
  for(int i = 0;i<number_Of_Motor;i++){ //iterate through all the motors
    dxl.ping(DXL_ID[i]);
    delay(100);
  
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_EXTENDED_POSITION);
    delay(100);
    dxl.torqueOn(DXL_ID[i]);
    delay(100);
    calibration[i] = (int32_t)dxl.getCurAngle(DXL_ID[i]);
    // print to monitor
    DEBUG_SERIAL.print(calibration[i]);
    if(i!=number_Of_Motor-1){
      DEBUG_SERIAL.print(", ");
    }
  }
  DEBUG_SERIAL.println("};");

  DEBUG_SERIAL.println("Started!");
  // cli.setOnError(errorCallback); // Set error Callback
  // Create the ping command with callback function
  /*
  run = cli.addCommand("run", pingCallback);
  stop =  cli.addCommand("stop", pingCallback);
  // Add argument with name "number", the value has to be set by the user
  // for example: echo -number 1
  run.addArgument("number");
  // for example: echo -str "pong"
    //              echo "pong"
  run.addPositionalArgument("str", "pong");
  // for example: echo -c
  run.addFlagArgument("c");

  stop.addArgument("number");
  stop.addPositionalArgument("str", "pong");
  stop.addFlagArgument("c");
*/
  // DEBUG_SERIAL.println("Type: run -str \"Hello World\" -number 1 -c");
  // DEBUG_SERIAL.println("Type: stop -str \"Hello World\" -number 1 -c");
  DEBUG_SERIAL.println("Type: \"run\" to start the worm");
  DEBUG_SERIAL.println("Type: \"stop\" to stop the worm");
}

void loop() {
  while(pause){
    delay(100);
    checkMonitorForInput();
  }
  // put your main code here, to run repeatedly:
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  
    
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
  iteration = 0;
  for(int i = 0;i<PERISTALSIS_CYCLES_NUMBER;i++){

    peristalsisRoutine (dxl, worm_pattern, number_Of_Motor, calibration, DXL_ID, iteration, full_contraction_peristalsis, !pause, turningrate);
    iteration++;
    iteration = iteration % peristalsis_cycle_size;
    DEBUG_SERIAL.print("Iteration Number: ");
    DEBUG_SERIAL.println(iteration);
    checkMonitorForInput();
    for(int i = 0;i<number_Of_Motor;i++){ //iterate through all the motors
    DEBUG_SERIAL.print(calibration[i]);
    if(i!=number_Of_Motor-1){
      DEBUG_SERIAL.print(", ");
    }
  }
  DEBUG_SERIAL.println("};");
      if(pause) {
      iteration--;
    }
  }
  
  iteration = 0;
  for(int i = 0;i<UNDULATION_CYCLES_NUMBER;i++){

    undulationRoutine (dxl, worm_pattern_turning, number_Of_Motor, calibration, DXL_ID, /*undulation_cycle_size-1-*/iteration, full_contraction_3D_peristalsis, !pause, turningrate);
    iteration++;
    iteration = iteration % undulation_cycle_size;
    DEBUG_SERIAL.print("Iteration Number: ");
    DEBUG_SERIAL.println(iteration);
    checkMonitorForInput();
    if(pause) {
      iteration--;
    }
  }
  delay(100);

  iteration = 0;
  for(int i = 0;i<TURNING_3D_CYCLES_NUMBER;i++){

    turning3DRoutine (dxl, worm_pattern_3D_turning, number_Of_Motor, calibration, DXL_ID, /*undulation_cycle_size-1-*/iteration, full_contraction_3D_turn, !pause, turningrate);
    iteration++;
    iteration = iteration % turning_3d_cycle_size;
    DEBUG_SERIAL.print("Iteration Number: ");
    DEBUG_SERIAL.println(iteration);
    checkMonitorForInput();
    if(pause) {
      iteration--;
    }
  }
  delay(100);

  iteration = 0;
  for(int i = 0;i<PERISTALSIS_3D_CYCLES_NUMBER;i++){
    peristalsis3DRoutine (dxl, worm_3D_pattern_peristalsis, number_Of_Motor, calibration, DXL_ID, /*undulation_cycle_size-1-*/iteration, full_contraction_3D_turn, !pause, turningrate);
    iteration++;
    iteration = iteration % peristalsis_3D_cycle_size;
    DEBUG_SERIAL.print("Iteration Number: ");
    DEBUG_SERIAL.println(iteration);
    checkMonitorForInput();
    if(pause) {
      iteration--;
    }
  }
  delay(100);

  iteration1 = 0;
  iteration2 = 0;
  for(int i = 0;i<UNDULATION_3D_OBSTACLE;i++){
    undulation_3D_Obstacle (dxl, worm_3D_pattern_lateral, worm_3D_pattern_vertical, number_Of_Motor, calibration, DXL_ID, /*undulation_cycle_size-1-*/iteration1, iteration2, /*vertical_deform, lateral_deform,*/ full_contraction_3D_undulation_obstacle, !pause, turningrate);
    iteration1++;
    iteration2++;
    iteration1 = iteration1 % obstacle_3D_cycle_lateral_size;
    iteration2 = iteration2 % obstacle_3D_cycle_vertical_size;
    DEBUG_SERIAL.print("Lateral Iteration Number: ");
    DEBUG_SERIAL.println(iteration1);
    DEBUG_SERIAL.print("Vertical Iteration Number: ");
    DEBUG_SERIAL.println(iteration2);
    checkMonitorForInput();
    if(pause) {
      iteration1--;
      iteration2--;
    }
  }
  delay(100);
}

void checkMonitorForInput(){
    // Check if user typed something into the serial monitor
    delay(100);
  if (DEBUG_SERIAL.available()) {
    // Read out string from the serial monitor
    String input = DEBUG_SERIAL.readStringUntil('\n');

    // Echo the user input
    DEBUG_SERIAL.print("# ");
    DEBUG_SERIAL.println(input);
    if (input == "run"){
      pause = false;
      DEBUG_SERIAL.println("worm is running");
    }
    if (input == "stop"){
      pause = true; //!pause;
      if(pause){
        DEBUG_SERIAL.println("worm is stopped");
      }
      else{
        DEBUG_SERIAL.println("worm is running");
      }
    }
    // Parse the user input into the CLI
    // cli.parse(input);
    // DEBUG_SERIAL.print("Porkboing");
  }
  if (cli.errored()) {
    CommandError cmdError = cli.getError();

    DEBUG_SERIAL.print("ERROR: ");
    DEBUG_SERIAL.println(cmdError.toString());

    if (cmdError.hasCommand()) {
        DEBUG_SERIAL.print("Did you mean \"");
        DEBUG_SERIAL.print(cmdError.getCommand().toString());
        DEBUG_SERIAL.println("\"?");
    }
  }
}

void relax_worm(){
  pause = !pause; //pause when button is pressed and unpaused when it's pressed again
  delay(1500);
  if(pause){
    // DEBUG_SERIAL.println("PAUSED");
  }
  else{
    // DEBUG_SERIAL.println("UNPAUSED");
  }
}