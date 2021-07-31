

bool RedDrop = false; 
bool BlueDrop = false;
bool GreenDrop = false;
bool YellowDrop = false;

bool inHallway = true ;
bool inRoom = false;

bool room1Complete = false;
bool room2Complete = false;
bool room3Complete = false;
bool room4Complete = false;

bool room1Enter = false;
bool room2Enter = false;
bool room3Enter = false;
bool room4Enter = false;

bool room1Exit = false;
bool room2Exit = false;
bool room3Exit = false;
bool room4Exit = false;




int roomID = 0;
int currentRoom;

#include <Servo.h>  // servo library
Servo servo1;  // servo control object
//360 servo setup
#include <NewPing.h> //include NewPing library
#include <Servo.h> //include Servo library
 int RForward = 0; 
 int RBackward =180 ; 
 int LForward = 0; 
 int LBackward = 180; 
 int BForward = 0; 
 int BBackward = 180;
 int RNeutral = 90; 
 int LNeutral = 90; 
 int BNeutral = 90;//constants for motor full speed
 int RForwardS = 87; 
 int RBackwardS =93 ; 
 int LForwardS = 87; 
 int LBackwardS = 93; 
 int BForwardS = 87; 
 int BBackwardS = 93;
 
 int RForwardM = 86; 
 int RBackwardM =94 ; 
 int LForwardM = 86; 
 int LBackwardM = 94; 
 int BForwardM = 86; 
 int BBackwardM = 94;
 int RNeutralS = 90; 
 int LNeutralS = 90; 
 int BNeutralS = 90;//constants SLOW motor speed
Servo leftMotor;
Servo rightMotor;
Servo backMotor;  //declare motors
// sonar setup
long durationF; 
long durationL;
long durationR;  //time it takes to recieve PING signal
//operational setup
const int dangerThresh = 7; //threshold for obstacles (in cm)
const int sideDangerThresh = 11 ;  
int distanceF, distanceR, distanceL; //distances on either side
// color sensor setup
#include <Wire.h>
#include <Math.h>
byte i2cWriteBuffer[10];
byte i2cReadBuffer[10];
#define SensorAddressWrite 0x29 //
#define SensorAddressRead 0x29 // 
#define EnableAddress 0xa0 // register address + command bits
#define ATimeAddress 0xa1 // register address + command bits
#define WTimeAddress 0xa3 // register address + command bits
#define ConfigAddress 0xad // register address + command bits
#define ControlAddress 0xaf // register address + command bits
#define IDAddress 0xb2 // register address + command bits
#define ColorAddress 0xb4 // register address + command bits



unsigned int clear_color = 0;
unsigned int red_color = 0;
unsigned int green_color = 0;
unsigned int blue_color = 0;

void setup()
{
 
displayColorCodes();
 //set push rod back to load hopper   
   int position;
   for(position = 300; position >0 ; position -= 2)
   {
    servo1.write(position);  // Move to next position
    delay(20);               // Short pause to allow it to move
   }   

 //360 servo setup
   Serial.begin(9600);   
   rightMotor.attach(10);
   leftMotor.attach(12);
   backMotor.attach(11);
 //color sensor setup
  Wire.begin();
  Serial.begin(9600);  // start serial for output
  init_TCS34725();
  get_TCS34725ID(); 

 // 180 Servo Setup
    servo1.attach(13, 900, 2100);  //(pin,min pulse,max pulse)


    currentRoom = 0;

    robotStop();
    delay(5000);
    getToYellow();
}



void loop()

//////////////////////////////////////////////////MAIN OPERATIONS LOOP//////////////////////////////////////////////////////////////////////////////////////
{
  working();
}  //end of loop
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




//*************************************************************FUNCTIONS****************************************************************************************

///////////////////ROBOT MOTION FUNCTIONS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void robotLeft()
{
 leftMotor.write(LForward); 
 rightMotor.write(RForward);
 backMotor.write(BForward);//turn left
}
void robotRight()
{
 leftMotor.write(LBackward); 
 rightMotor.write(RBackward);
 backMotor.write(BBackward);//turn right
}
void robotDriftLeft()
{
    leftMotor.write(LNeutral);  // drift left 
    rightMotor.write(RForward);
    backMotor.write(BNeutral);
}
void robotDriftRight()
{
   leftMotor.write(LBackward);  // drift right
   rightMotor.write(RNeutral);
   backMotor.write(BNeutral);
}
void robotStop()
{
  leftMotor.write(LNeutral);  // stop robot
  rightMotor.write(RNeutral);
  backMotor.write(BNeutral);
}
void robotForward()
{
 leftMotor.write(LBackward);  // forward
 rightMotor.write(RForward);
 backMotor.write(BNeutral);
 
}
void robotSForward()
{
 leftMotor.write(LBackwardS);  // forward
 rightMotor.write(RForwardS);
 backMotor.write(BNeutralS);
}

void robotForwardM()
{
 leftMotor.write(LBackwardM);  // forward
 rightMotor.write(RForwardM);
 backMotor.write(BNeutral);
}

void robotSBackward()
{
 leftMotor.write(LForwardS);  // backward slow
 rightMotor.write(RBackwardS);
 backMotor.write(BNeutralS);  
}
void robotBackward()
{
 leftMotor.write(LForward);  // backward
 rightMotor.write(RBackward);
 backMotor.write(BNeutral);  
}
void pivotForward()
{
 driftRight();
 delay(1500);
 driftLeft();
 delay(1500);
}
void driftLeft()
{

 leftMotor.write(LBackward); 
 rightMotor.write(RForwardS);
 backMotor.write(BNeutralS);
 
}
void driftRight()
{
 leftMotor.write(LBackwardS);  
 rightMotor.write(RForward);
 backMotor.write(BNeutralS);

}
/////////////////////////////SONAR FUNCTIONS//////////////////////////////////////////////////////////////////////////////////////////////

long showDistance()
{
    Serial.print("DistanceF: ");
    Serial.print(distanceF);
    Serial.print(" \tDistanceR: ");
    Serial.print(distanceR);
    Serial.print(" \tDistanceL: ");
    Serial.println(distanceL);
} 
long pingF()
{
  const int trigPinF = 3;
  const int echoPinF = 2;   //set up pins
  pinMode(trigPinF, OUTPUT);
  digitalWrite(trigPinF, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinF, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPinF, LOW);
  
  //Get duration it takes to receive echo
  pinMode(echoPinF, INPUT);
  durationF = pulseIn(echoPinF, HIGH);
  
  //Convert duration into distance
  return (durationF / 29 / 2);
}
long pingL()
{
  const int trigPinL = 5;
  const int echoPinL = 4; //set up pins
  pinMode(trigPinL, OUTPUT);
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPinL, LOW);
  pinMode(echoPinL, INPUT);       //Get duration it takes to receive echo
  durationL = pulseIn(echoPinL, HIGH);
  return (durationL / 29 / 2);   //Convert duration into distance
}
long pingR()
{
  const int trigPinR = 7;
  const int echoPinR = 6; //set up pins  
  pinMode(trigPinR, OUTPUT);
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPinR, LOW);
  pinMode(echoPinR, INPUT);            //Get duration it takes to receive echo
  durationR = pulseIn(echoPinR, HIGH);
  return (durationR / 29 / 2);       //Convert duration into distance

}

void getDistance()
{
     distanceF = pingF();
     distanceL = pingL();
     distanceR = pingR();
     showDistance();
}


//////////////////////////COLOR SENSOR CODE///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void displayColorCodes()
{
  Serial.print("clear color=");           // shows value number for each color value
  Serial.print(clear_color, DEC);    
  Serial.print(" red color=");
  Serial.print(red_color, DEC);    
  Serial.print(" green color=");
  Serial.print(green_color, DEC);    
  Serial.print(" blue color=");
  Serial.println(blue_color, DEC);
}

void Writei2cRegisters(byte numberbytes, byte command)
{
    byte i = 0;
    Wire.beginTransmission(SensorAddressWrite);   // Send address with Write bit set
    Wire.write(command);                          // Send command, normally the register address 
    for (i=0;i<numberbytes;i++)                       // Send data 
    {
    Wire.write(i2cWriteBuffer[i]);
    Wire.endTransmission();
    delayMicroseconds(100);      // allow some time for bus to settle  
    }    
}
byte Readi2cRegisters(int numberbytes, byte command)
{
   byte i = 0;
   Wire.beginTransmission(SensorAddressWrite);   // Write address of read to sensor
   Wire.write(command);
   Wire.endTransmission();
   delayMicroseconds(100);      // allow some time for bus to settle      
   Wire.requestFrom(SensorAddressRead,numberbytes);   // read data
   for(i=0;i<numberbytes;i++)
   {
   i2cReadBuffer[i] = Wire.read();
   Wire.endTransmission();   
   delayMicroseconds(100);      // allow some time for bus to settle      
   }
}  
void init_TCS34725(void)
{
  i2cWriteBuffer[0] = 0x10;
  Writei2cRegisters(1,ATimeAddress);    // RGBC timing is 256 - contents x 2.4mS =  
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1,ConfigAddress);   // Can be used to change the wait time
  i2cWriteBuffer[0] = 0x00;
  Writei2cRegisters(1,ControlAddress);  // RGBC gain control
  i2cWriteBuffer[0] = 0x03;
  Writei2cRegisters(1,EnableAddress);    // enable ADs and oscillator for sensor  
}
void get_TCS34725ID(void)
{
  Readi2cRegisters(1,IDAddress);
  if (i2cReadBuffer[0] = 0x44)
  {
  Serial.println("TCS34725 is present");  
  }  
  else
  {
  Serial.println("TCS34725 not responding");    
  }
}
void get_Colors(void)
{
 
  Readi2cRegisters(8,ColorAddress);
  readColorValues();
 // yellowTape Detector
  if(clear_color >= 6000&& clear_color <= 8841 && red_color >= 2138 && red_color <= 3906 && green_color >= 1819 && green_color <= 3376 && blue_color >= 797 && blue_color <= 1546)
   {
    Serial.print ("on YellowTape ");
    roomID = 4;
   }
   
 //greenTape Detector
   if(clear_color >= 1025&& clear_color <= 2192 && red_color >= 278 && red_color <= 482 && green_color >= 360 && green_color <= 517 && blue_color >= 272 && blue_color <= 426)
   {
    Serial.print ("on GreenTape ");
    roomID = 3;
   }
 
 //whiteTape detector
   if(red_color >= 1000 && green_color >= 1000 && blue_color >= 1000)
   {
    Serial.print ("on WhiteTape ");
    
   }
  
 //redTapeDetector
   if(clear_color >= 1800 && clear_color <= 3500 && red_color >= 1000 && red_color <= 2500 && green_color >= 325 && green_color <= 750 && blue_color >= 300 && blue_color <= 650)
   {
    Serial.print ("on RedTape ");
    roomID = 1;
   }
 
 //blueTapeDetector
   if(clear_color >= 1691&& clear_color <= 2126 && red_color >= 360 && red_color <= 566 && green_color >= 474 && green_color <= 696 && blue_color >= 746 && blue_color <= 986)
   {
    Serial.print ("on BlueTape ");
    roomID = 2;
   }
  
 //blackFloorDerector
   if(clear_color >= 0 && clear_color <= 1000 && red_color >= 0 && red_color <= 500 && green_color >= 0 && green_color <= 500 && blue_color >= 0 && blue_color <= 500)
   {
    Serial.print ("on Black Floor   ");
   }
  
 // target detector
   if(clear_color >= 1 && clear_color <= 12000 && red_color >= 3000 && red_color <= 5000 && green_color >= 1300 && green_color <= 3000 && blue_color >= 600 && blue_color <= 2000)
   {
   Serial.print (" on target   ");
   }
   else
   {
   Serial.print ("not on target   ");
   }

 displayColorCodes();


    Serial.println(roomID);
    Serial.print( "= room ID");
    Serial.println(currentRoom);
    Serial.print( "= current room");
}

void readColorValues()
{
  clear_color = (unsigned int)(i2cReadBuffer[1]<<8) + (unsigned int)i2cReadBuffer[0]; //******** these variables needed to be declared in loop before using if statement
  red_color = (unsigned int)(i2cReadBuffer[3]<<8) + (unsigned int)i2cReadBuffer[2];
  green_color = (unsigned int)(i2cReadBuffer[5]<<8) + (unsigned int)i2cReadBuffer[4];
  blue_color = (unsigned int)(i2cReadBuffer[7]<<8) + (unsigned int)i2cReadBuffer[6];
}
  
///////////////////////////////////////OPERATIONAL FUNCTIONS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stayInRoom() // robot goes into slow mode to pick up colors better  
{
   
   inRoom = true;
   inHallway = false;
   get_Colors();
   if ( (2000 > clear_color && red_color > blue_color && red_color > 1000 && currentRoom ==1)||(7000 > clear_color && blue_color > red_color && blue_color > 1000 && currentRoom ==2)||( clear_color >= 1025&& clear_color <= 2192 && red_color >= 278 && red_color <= 482 && green_color >= 360 && green_color <= 517 && blue_color >= 272 && blue_color <= 426 && currentRoom == 3)||(
  clear_color >= 4906&& clear_color <= 8841 && red_color >= 2138 && red_color <= 3906 && green_color >= 1819 && green_color <= 3376 && blue_color >= 797 && blue_color <= 1546&& currentRoom == 4))//sees color tape ture around
   {
    robotStop();          //stop reverse turn 90 degrees(stay in room)
    delay(500);
    robotBackward();
    delay(1000);
    distanceL = pingL();
    distanceR = pingR();
    if (distanceL>distanceR) //if left is less obstructed 
    {
     leftMotor.write(LForward); 
     rightMotor.write(RForward);
     backMotor.write(BForward);//turn left
     delay(2000); 
     Serial.print("right obstruction");
     robotStop();
     delay (500);
    }
    else if (distanceR>distanceL) //if right is less obstructed
    {
     leftMotor.write(LBackward); 
     rightMotor.write(RBackward);
     backMotor.write(BBackward);//turn right
     delay(2000);
     robotStop();
     delay(500);
     Serial.print("Left obstruction");
    }
    else //if they are equally obstructed
    {
     leftMotor.write(LForward); 
     rightMotor.write(RForward); 
     backMotor.write(BForward);//turn 180 degrees
     delay(2000);
     Serial.print("equal onstruction");
    }
     get_Colors();
     stayInRoom();
    }

   else if (clear_color >= 1 && clear_color <= 12000 && red_color >= 3000 && red_color <= 5000 && green_color >= 1300 && green_color <= 3000 && blue_color >= 600 && blue_color <= 2000)//on target
   {
    pillDrop();
   }
   else if(inRoom == true)
   {
    slowAvoidWalls();
    get_Colors();
    Serial.println("/////////////////STAY IN ROOM////////////////////");
    stayInRoom();
   }
}
void avoidWalls()
{
    distanceF = pingF();
    distanceR = pingR();
    distanceL = pingL();
    if(distanceL > dangerThresh && distanceR > dangerThresh)
    {
     forwardScan();
     get_Colors();
    }
    else if (distanceL < sideDangerThresh && distanceR > sideDangerThresh)
    {
     leftMotor.write(LBackward);  // drift right away from wall
     rightMotor.write(RNeutral);
     backMotor.write(BNeutral);
     Serial.print("  avoiding left wall  ");
    }
    else if (distanceL >  sideDangerThresh && distanceR < sideDangerThresh)
    {
     leftMotor.write(LNeutral);  // drift left away from wall
     rightMotor.write(RForward);
     backMotor.write(BNeutral);
     Serial.print("  avoiding right wall  ");
    }
    else
    {
     leftMotor.write(LForward);   // full reverse
     rightMotor.write(RBackward);
     backMotor.write(BNeutral);
     delay(500);
     compareDistance();
     Serial.print("  avoiding both walls  ");
    }
}
void pillDrop()
{ 
  
    int position;
    leftMotor.write(LNeutral);  // stop robot
    rightMotor.write(RNeutral);
    backMotor.write(BNeutral);
    Serial.println ("///////////PILL DROP////////////");
    int i;
    for(i=0; i<1; i++)
    { 
      for(position = 300; position >0 ; position -= 2)
      {
      servo1.write(position);  // Move to next position
      delay(20);               // Short pause to allow it to move
      }
    }  

    dropFlagSelect();
    distanceL = pingL();
    distanceR = pingR();
    if (distanceL>distanceR) //if left is less obstructed 
    {
     robotLeft();
     delay(2000);
     robotStop();
     delay(500); 
     leftMotor.write(LNeutral); 
     rightMotor.write(RForward);
     backMotor.write(BBackward);//diagonal left
     delay(1000); 
     Serial.print("right obstruction");
     robotStop();
     delay (500);
    }
    else if (distanceR>distanceL) //if right is less obstructed
    {
     robotRight();
     delay(2000);
     robotStop();
     delay(500);
     leftMotor.write(LBackward); 
     rightMotor.write(RNeutral);
     backMotor.write(BForward);//diagonal right
     delay(1000);
     robotStop();
     delay(500);
     Serial.print("Left obstruction");
    }
    else //if they are equally obstructed
    {
     leftMotor.write(LForward); 
     rightMotor.write(RForward); 
     backMotor.write(BForward);//turn 180 degrees
     delay(1000);
     Serial.print("equal onstruction");
    }
    
 leaveRoom();
    
}
void forwardScan() 
{ 
   distanceF = pingF();
   if (distanceF>dangerThresh) //if path is clear
   {
    robotForward();
    Serial.print("  clear path  ");
   }
    else //if path is blocked
    {
     robotStop();
     delay(1000); 
     compareDistance();
     Serial.print("  obstruction front  ");
    }
     distanceL = pingL();
     distanceR = pingR();
}   

void compareDistance()
{
   distanceL = pingL();
   distanceR = pingR();
   if (distanceL>distanceR) //if left is less obstructed 
   {
    leftMotor.write(LForward); 
    rightMotor.write(RForward);
    backMotor.write(BForward);//turn left
    delay(800); 
    Serial.print("right obstruction");
    robotStop();
    delay (500);
   }
   else if (distanceR>distanceL) //if right is less obstructed
   {
    leftMotor.write(LBackward); 
    rightMotor.write(RBackward);
    backMotor.write(BBackward);//turn right
    delay(800);
    robotStop();
    delay(500);
    Serial.print("Left obstruction");
   }
   else //if they are equally obstructed
   {
    leftMotor.write(LForward); 
    rightMotor.write(RForward); 
    backMotor.write(BForward);//turn 180 degrees
    delay(2000);
    Serial.print("equal onstruction");
   }
}

void slowForwardScan()
{
   distanceF = pingF();
   if (distanceF>dangerThresh) //if path is clear
   {
    robotSForward();
    Serial.print("  clear path  ");
    }
   else //if path is blocked
   {
    leftMotor.write(LNeutral);
    rightMotor.write(RNeutral);
    backMotor.write(BNeutral);
    delay(1000); 
    compareDistance();
    Serial.print("  obstruction front  ");
   }
   distanceL = pingL();
   distanceR = pingR();
}
void slowAvoidWalls()
{
 distanceF = pingF();
 distanceR = pingR();
 distanceL = pingL();
 if(distanceL > dangerThresh && distanceR > dangerThresh)
 {
  slowForwardScan();
 }
 else if (distanceL < sideDangerThresh && distanceR > sideDangerThresh)
 {
  leftMotor.write(LBackwardS);  // drift right away from wall
  rightMotor.write(RNeutralS);
  backMotor.write(BNeutralS);
  Serial.print("  avoiding left wall  ");
 }
 else if (distanceL >  sideDangerThresh && distanceR < sideDangerThresh)
 {
  leftMotor.write(LNeutralS);  // drift left away from wall
  rightMotor.write(RForwardS);
  backMotor.write(BNeutralS);
  Serial.print("  avoiding right wall  ");
 }
 else
 {
  leftMotor.write(LForwardS);   // full reverse
  rightMotor.write(RBackwardS);
  backMotor.write(BNeutralS);
  leftMotor.write(LForward);  
  rightMotor.write(RForward); 
  backMotor.write(BForward);//turn 90 degrees
  delay(1200);
  Serial.print("  avoiding both walls  ");
 }
}

void leaveRoom() //after dropping pill seach for exit of room and do not perform any other actions
{
   displayColorCodes();
  if((2000 > clear_color && red_color > blue_color && red_color > 1000 && currentRoom ==1)||(7000 > clear_color && blue_color > red_color && blue_color > 1000 && currentRoom ==2)||( clear_color >= 1025&& clear_color <= 2192 && red_color >= 278 && red_color <= 482 && green_color >= 360 && green_color <= 517 && blue_color >= 272 && blue_color <= 426 && currentRoom == 3)||(
   clear_color >= 4906&& clear_color <= 8841 && red_color >= 2138 && red_color <= 3906 && green_color >= 1819 && green_color <= 3376 && blue_color >= 797 && blue_color <= 1546&& currentRoom == 4))
   {
    exitRoom();
   }
   else
   {
   slowAvoidWalls();
   get_Colors();
   Serial.println("   ///////////////////////leave///////////////  ");
   leaveRoom();
   }
}

void exitRoom()
{
 
 robotStop();
 delay(500);
 robotForward();
 delay(500);
 Serial.println("****************************************************************************************************************EXIT");
 inHallway = true;
 inRoom = false;
 roomID = 0;
 currentRoom = 0;
 exitFlagSelect();
 robotForward();
 delay(1000);
 get_Colors();
 working();
 

}  


void working()
{
 get_Colors();
 if(inHallway == true && RedDrop == false && roomID == 1 || inHallway == true && BlueDrop == false && roomID == 2 || inHallway == true && GreenDrop == false && roomID == 3|| inHallway == true && YellowDrop == false && roomID == 4)
 {
  get_Colors();
  enterFlagSelect();
  robotForward();
  delay(1000);
  stayInRoom();
 }

 else if( (inHallway == true && room1Exit == true && roomID == 1) || (inHallway == true && room2Exit == true && roomID == 2) || (inHallway == true && room3Exit == true && roomID == 3)|| (inHallway == true && room4Exit == true && roomID == 4))
 {
    robotStop();          //stop reverse turn 90 degrees(stay in room)
    delay(500);
    Serial.println("NOT THAT ROOM!!!");
    robotBackward();
    delay(2000);
    compareDistance();
    get_Colors();
    roomID=0;
    working();
    
 }
 else if (inHallway == true)
 {
  hallwayMode();
  working();
 
 }
}


void hallwayMode()
{
 stopOnWhite();
 get_Colors();
 displayFlags();
 Serial.println ("####################################HALWAY###########################################");
 
  
 }



/////////////////////////////////////////////////////////////////////////FLAGS///////////////////////////////////////////////////////////////////////////

void dropFlagSelect()
{
 if(room1Enter == true)
 {
  RedDrop = true;
  Serial.println("REDDROP = TRUE");
  currentRoom == 1;
 } 
 if(room2Enter == true)
 {
  BlueDrop = true;
  Serial.println("BLUEDROP = TRUE");
  currentRoom =2;
 } 
 if (room3Enter == true)
 {
  GreenDrop = true;
  Serial.println("GREENDROP = TRUE"); 
  currentRoom = 3; 
 } 
 if (room4Enter == true)
 {
  YellowDrop = true;
  Serial.println("YELLOWDROP = TRUE");
  currentRoom =4;
 } 
}  

void enterFlagSelect()
{
 if(roomID == 1)
 {
  room1Enter = true;
  Serial.println("ROOM1ENTER = TRUE");
  currentRoom = 1;
 } 
 if(roomID == 2) 
 {
  room2Enter = true;
  Serial.println("ROOM2ENTER = TRUE");
  currentRoom = 2;
 } 
 if (roomID == 3)
 {
  room3Enter = true;
  Serial.println("ROOM3ENTER = TRUE");
  currentRoom = 3;
 } 
 if (roomID == 4)
 {
  room4Enter = true;
  Serial.println("ROOM4ENTER = TRUE");
  currentRoom = 4;
 } 
}  

void exitFlagSelect()
{
  if (room1Enter == true && RedDrop == true)
  {
   room1Exit = true;
   Serial.println("ROOM1EXIT = TRUE");
  } 
  if (room2Enter == true && BlueDrop == true)
  {
   room2Exit = true;
   Serial.println("ROOM2EXIT = TRUE");
  }
  if (room3Enter == true && GreenDrop == true )
  {
   room3Exit = true;
   Serial.println("ROOM3EXIT = TRUE");
  }
  if (room4Enter == true && YellowDrop == true)
  {
   room4Exit = true;
   Serial.println("ROOM4EXIT = TRUE");
  }
}




void displayFlags()
{
  if (room1Exit == true)//red
  {
    Serial.println("room 1 complete");
  }
   else if (room1Exit == false)
  {
    Serial.println("room 1 incomplete");
  }
  if (room2Exit == true)//blue
  {
    Serial.println("room 2 complete");
  }
  else if (room2Exit == false)
  {
    Serial.println("room 2 incomplete");
  }
  if (room3Exit == true)//green
  {
    Serial.println("room 3 complete");
  }
   else if(room3Exit == false)
  {
    Serial.println("room 3 incomplete");
  }
  if (room4Exit == true)//yellow
  {
    Serial.println("room 4 complete");
  }  
  else if(room4Exit == false)
  {
    Serial.println("room 4 incomplete");
  }

    
  
}

void midAvoidWalls()
{
 distanceF = pingF();
 distanceR = pingR();
 distanceL = pingL();
 if(distanceL > dangerThresh && distanceR > dangerThresh)
 {
  midForwardScan();
 }
 else if (distanceL < sideDangerThresh && distanceR > sideDangerThresh)
 {
  leftMotor.write(LBackwardM);  // drift right away from wall
  rightMotor.write(RNeutralS);
  backMotor.write(BNeutralS);
  Serial.print("  avoiding left wall  ");
 }
 else if (distanceL >  sideDangerThresh && distanceR < sideDangerThresh)
 {
  leftMotor.write(LNeutralS);  // drift left away from wall
  rightMotor.write(RForwardM);
  backMotor.write(BNeutralS);
  Serial.print("  avoiding right wall  ");
 }
 else
 {
  leftMotor.write(LForwardM);   // full reverse
  rightMotor.write(RBackwardM);
  backMotor.write(BNeutralS);
  leftMotor.write(LForward);  
  rightMotor.write(RForward); 
  backMotor.write(BForward);//turn 90 degrees
  delay(1200);
  Serial.print("  avoiding both walls  ");
 }
}

void midForwardScan() 
{ 
   distanceF = pingF();
   if (distanceF>dangerThresh) //if path is clear
   {
    robotForwardM();
    Serial.print("  clear path  ");
   }
    else //if path is blocked
    {
     robotStop();
     delay(1000); 
     compareDistance();
     Serial.print("  obstruction front  ");
    }
     distanceL = pingL();
     distanceR = pingR();
}   


void stopOnWhite() // robot goes into slow mode to pick up colors better  
{
   
   
   if (red_color >= 1000 && green_color >= 1000 && blue_color >= 1000)//on white tape
   {
      robotStop();
      delay(1000);
      readTape();
      colorCheck();
      
   }
   else
   {
    midAvoidWalls();
   }


}

void colorCheck()
{

      if(clear_color >= 0 && clear_color <= 1000 && red_color >= 0 && red_color <= 500 && green_color >= 0 && green_color <= 500 && blue_color >= 0 && blue_color <= 500)//on black
      {
         jumpBackward();

      }
      else if (red_color >= 1000 && green_color >= 1000 && blue_color >= 1000)//on white tape
      {
         jumpForward();

      }

      else if((clear_color >= 1800 && clear_color <= 3500 && red_color >= 1000 && red_color <= 2500 && green_color >= 325 && green_color <= 750 && blue_color >= 300 && blue_color <= 650)|| (clear_color >= 1691&& clear_color <= 2126 && red_color >= 360 && red_color <= 566 && green_color >= 474 && green_color <= 696 && blue_color >= 746 && blue_color <= 986)|| (clear_color >= 4906&& clear_color <= 8841 && red_color >= 2138 && red_color <= 3906 && green_color >= 1819 && green_color <= 3376 && blue_color >= 797 && blue_color <= 1546)|| (clear_color >= 1025&& clear_color <= 2192 && red_color >= 278 && red_color <= 482 && green_color >= 350 && green_color <= 517 && blue_color >= 272 && blue_color <= 426))
      {
         
      
       get_Colors();
       working();
      }

      else
      {
        robotBackward();
        delay(1000);
        stopOnWhite();
      }

      
}



void jumpBackward()
{
int i;  
for(i=0; i<=5; i++)
 { 
  if((clear_color >= 1800 && clear_color <= 3500 && red_color >= 1000 && red_color <= 2500 && green_color >= 325 && green_color <= 750 && blue_color >= 300 && blue_color <= 650)|| (clear_color >= 1691&& clear_color <= 2126 && red_color >= 360 && red_color <= 566 && green_color >= 474 && green_color <= 696 && blue_color >= 746 && blue_color <= 986)|| (clear_color >= 4906&& clear_color <= 8841 && red_color >= 2138 && red_color <= 3906 && green_color >= 1819 && green_color <= 3376 && blue_color >= 797 && blue_color <= 1546)|| (clear_color >= 1025&& clear_color <= 2192 && red_color >= 278 && red_color <= 482 && green_color >= 350 && green_color <= 517 && blue_color >= 272 && blue_color <= 426))
  {
 
  

  get_Colors();
  working();
  }
  else
  {
  robotBackward();
  delay(100);
  robotStop();
  delay(500);
  readTape();

  }  
 }
 
 
 robotStop();
 delay(500);
 stopOnWhite();
 
}



void jumpForward()
{
 int i;

  for(i=0; i<=5; i++)  
  { 
   if((clear_color >= 1800 && clear_color <= 3500 && red_color >= 1000 && red_color <= 2500 && green_color >= 325 && green_color <= 750 && blue_color >= 300 && blue_color <= 650)|| (clear_color >= 1691&& clear_color <= 2126 && red_color >= 360 && red_color <= 566 && green_color >= 474 && green_color <= 696 && blue_color >= 746 && blue_color <= 986)|| (clear_color >= 4906&& clear_color <= 8841 && red_color >= 2138 && red_color <= 3906 && green_color >= 1819 && green_color <= 3376 && blue_color >= 797 && blue_color <= 1546)|| (clear_color >= 1025&& clear_color <= 2192 && red_color >= 278 && red_color <= 482 && green_color >= 350 && green_color <= 517 && blue_color >= 272 && blue_color <= 426))
   {
  
 
 
   get_Colors();
   working();
   }
   else
   {
   robotForward();
   delay(100);
   robotStop();
   delay(500);
   readTape();
   }  
  }  
  

 robotStop();
 delay(500);
 robotBackward();
 delay(600);
 robotStop();
 delay(500);
 stopOnWhite();
 
}


void readTape()
{
      Serial.println("reading tape");
      get_Colors();
      get_Colors();
      get_Colors();
      get_Colors();
      get_Colors();
      
}



void safeNav()
{
  long unsigned int time = millis;
 
  if(time <= 3000)
  {
    robotForward;
    Serial.println(time);
  }
  else if (time >= 3000 && time <= 3500)
  {
    robotStop();
     Serial.println(time);
  }
  else if (time >= 3500 && time <= 4000)
  {
    robotLeft();
     Serial.println(time);
  }
  else if (time >= 4000 && time <= 4500)
  {
    robotStop();
     Serial.println(time);
  }
  else if (time >= 4500 && time <= 5000)
  {
    robotRight();
     Serial.println(time);
  }
  else if (time >= 5000 && time <= 5500)
  {
    robotStop();
     Serial.println(time);
  }
  else if (time >= 5500 && time <= 6000)
  {
    robotRight();
     Serial.println(time);
  }
  else if (time >= 6000 && time <= 6500)
  {
    robotStop();
     Serial.println(time);
  }
  else if (time >= 6500 && time <= 7000)
  {
    robotLeft();
     Serial.println(time);
  }
  else if (time >= 7000 && time <= 7500)
  {
    robotStop();
     Serial.println(time);
    
  }
  else
  {
    time = time - time;
     Serial.println(time);
  }
    
  
}


void getToYellow()
{
  robotForward();
  delay(6000);
  robotStop();
  delay(500);
  robotRight();
  delay(1100);
  robotStop();
  delay(500);
  robotForward();
  delay(4000);
  robotStop();
  delay(500);
  robotRight();
  delay(1100);
  robotStop();
  delay(500);
}


//**************************************************************************************************************************************************************

////////////////////////////COMMENTS//////////////////////////////////////////////
//(clear_color >= 500 && clear_color <= 5000 && red_color >= 200 && red_color <= 1000 && green_color >= 400 && green_color <= 2100 && blue_color >= 200 && blue_color <= 1500) = YELLOWTAPE
//(clear_color >= 500 && clear_color <= 5000 && red_color >= 200 && red_color <= 1000 && green_color >= 400 && green_color <= 2100 && blue_color >= 200 && blue_color <= 1500) === GREENTAPE
//(red_color >= 1000 && green_color >= 1000 && blue_color >= 1000) =============================================================================================================== WHITETAPE
//(clear_color >= 500 && clear_color <= 10000 && red_color >= 1000 && red_color <= 2400 && green_color >= 200 && green_color <= 1500 && blue_color >= 200 && blue_color <= 1500) = REDTAPE
//(clear_color >= 1000 && clear_color <= 3000 && red_color >= 0 && red_color <= 1000 && green_color >= 100 && green_color <= 900 && blue_color >= 800 && blue_color <= 1200) ===== BLUETAPE
//(clear_color >= 0 && clear_color <= 1000 && red_color >= 0 && red_color <= 500 && green_color >= 0 && green_color <= 500 && blue_color >= 0 && blue_color <= 500) ============== BLACKFLOOR
//(clear_color >= 1 && clear_color <= 12000 && red_color >= 3000 && red_color <= 5000 && green_color >= 1300 && green_color <= 3000 && blue_color >= 600 && blue_color <= 2000)=== TARGET
//
//
//
//

/*
 new color tape
 
 (clear_color >= 1800 && clear_color <= 3500 && red_color >= 1000 && red_color <= 2500 && green_color >= 325 && green_color <= 750 && blue_color >= 300 && blue_color <= 650)
 window Red

  (clear_color >= 1691&& clear_color <= 2126 && red_color >= 360 && red_color <= 566 && green_color >= 474 && green_color <= 696 && blue_color >= 746 && blue_color <= 986)
 window Blue

  (clear_color >= 4906&& clear_color <= 8841 && red_color >= 2138 && red_color <= 3906 && green_color >= 1819 && green_color <= 3376 && blue_color >= 797 && blue_color <= 1546)
 window Yellow

  (clear_color >= 1025&& clear_color <= 2192 && red_color >= 278 && red_color <= 482 && green_color >= 360 && green_color <= 517 && blue_color >= 272 && blue_color <= 426)
 window Green
 */


