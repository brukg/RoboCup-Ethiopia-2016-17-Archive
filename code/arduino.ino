#include "SoftwareSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#define BUFFER_SIZE 16//This will prevent buffer overruns.
#define RSTurnRight       0x80
#define RSRightArmUp      0x81
#define RSRightArmOut     0x82
#define RSTiltBodyRight   0x83
#define RSRightArmDown    0x84
#define RSRightArmIn      0x85
#define RSWalkForward     0x86
#define RSWalkBackward    0x87
#define RSTurnLeft        0x88
#define RSLeftArmUp       0x89
#define RSLeftArmOut      0x8A
#define RSTiltBodyLeft    0x8B
#define RSLeftArmDown     0x8C
#define RSLeftArmIn       0x8D
#define RSStop            0x8E
#define RSWakeUp          0xB1
#define RSBurp            0xC2
#define RSRightHandStrike 0xC0
#define RSNoOp            0xEF
#define LeanForward       0xAD
#define stepForward       0xA6
#define stepLeft          0xA8
#define stepright         0xA0
#define reset             0xAE
#define stepBackward      0xA7
#define bulldozer         0xC6
SoftwareSerial serial_connection(10, 11);//Create a serial connection with TX and RX on these pins
Servo pan;
Servo tilt;
Servo hit;


int countTilt=0;
int countSearch=0;
int pospan = 90;
int postilt=90;
int poshit=140;
int j;
int IROut=3;
int forward,backward;
int pancond2=0, pancond1=0, tiltcond=0;
int rotateValue=0;
int flagRotate=0;
int bitTime=516;
int left = 0,right = 0;
int headingDegrees;
int currentVal=0;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
const int tsDelay = 833; // us, as estimated
String inData;//This is a character buffer where the data sent by the python script will go.
int inDataFo=0;
char inChar=0;//Initialie the first character as nothing
int count=0;//This is the number of lines sent in from the python script
int i=0;//Arduinos are not the most capable chips in the world so I just create the looping variable once
boolean a=0;
boolean chck=0;
void delayTs(unsigned int slices)
{
  delayMicroseconds(tsDelay * slices);
}


// send the whole 8 bits
void RSSendCommand(int cmd)
{
  digitalWrite(IROut, LOW);
  delayTs(8);
  for(char b = 7; b>=0; b--) {
    digitalWrite(IROut, HIGH);
    delayTs( (cmd & 1<<b) ? 4 : 1 );
    digitalWrite(IROut, LOW);
    delayTs(1);
  }
  digitalWrite(IROut, HIGH);
}

void localsetup(int input){
  pospan = 90;
  postilt = 90;
  poshit = 140;
  pan.write(pospan);
  delay(30);
  tilt.write(postilt);
  delay(30);
  hit.write(poshit);
  delay(30);
  Serial.begin(9600);
  Serial.println(input);
  Serial.println("calibration started");
  forward = input;
  //Serial.println("forward " +forward);
  backward=(input+180)% 360;
  //Serial.println("backward " +backward);
  left = (input+270) % 360;
  //Serial.println("left "+left);
  right = (input+90)%360;
  //Serial.println("right "+right);
  Serial.println("calibration finished");

}
void updatedegree(){
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22;
  heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI;

}

void setup()
{
  pan.attach(8);
  tilt.attach(9);
  hit.attach(7);
  pan.write(pospan);
  delay(30);
  tilt.write(postilt);
  delay(30);
  hit.write(poshit);
  delay(30);
  pinMode(IROut, OUTPUT);
  serial_connection.begin(9600);
  Serial.begin(9600);
  Serial.println("Started");//Tell the serial monitor that the sketch has started.
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    //while(1);
  }

}
void loop()
{
  if(flagRotate == 1 || flagRotate == -1){
  countTilt += 1;
   updatedegree();
   Serial.print(currentVal);Serial.print(" ");Serial.print(String(headingDegrees));Serial.print(" ");Serial.println(String(countTilt));

   if(abs(rotateValue - headingDegrees) <= 5){
    RSSendCommand(RSStop);
    serial_connection.println("S");
    Serial.println("S");
    flagRotate=0;
    rotateValue=0;
   }
   else if(countTilt > 30  and abs(currentVal- headingDegrees) <= 2){
     RSSendCommand(RSStop);
     serial_connection.println("s");
     Serial.println("s");
     flagRotate=0;
     rotateValue=0;
     countTilt=0;
   }

  }

byte byte_count=serial_connection.available();//This gets the number of bytes that were sent by the python script
delay(30);
//hit.detach();
//tilt.detach();
delay(10);
if(byte_count)//If there are any bytes then deal with them
{
Serial.println("Incoming Data");//Signal to the monitor that something is happening
int first_bytes=byte_count;//initialize the number of bytes that we might handle.
int remaining_bytes=0;//Initialize the bytes that we may have to burn off to prevent a buffer overrun
  if(first_bytes>=BUFFER_SIZE-1)//If the incoming byte count is more than our buffer...
  {
  remaining_bytes=byte_count-(BUFFER_SIZE-1);
  }
  for(i=0;i<first_bytes;i++)
  {
  inChar=serial_connection.read();
  inData += inChar;
  }
  //inData='\0';
  Serial.println(inData);

  //hit.attach(9);
  //tilt.attach(5);
  Serial.println(countSearch);
  if(String(inData[0]) == "n"){
  countSearch += 1 ;
  }else{
  countSearch = 0;
  }


  if(String(inData[0])=="!")
  {
    RSSendCommand(reset);
  }
  if(String(inData[0])=="C")//This could be any motor start string we choose from the python script
  {
    updatedegree();
    localsetup(headingDegrees);
  }
  if(String(inData[0])=="c")//This could be any motor start string we choose from the python script
  {
    pospan=90;
    postilt=90;
    pan.write(90);
    tilt.write(90);
    Serial.println("centered");
  }
  if(String(inData[0])=="h")//This could be any motor start string we choose from the python script
  {
    poshit = 20;
    delay(10);
    hit.write(poshit);
    delay(100);
    poshit = 140;
    delay(100);
    hit.write(poshit);
  }
  else if(String(inData[0])=="Z")//This could be any motor start string we choose from the python script
   {
     String in=inData;
     int num=in.substring(2).toInt();
     Serial.println("Here is The Value-Z");
     Serial.println(String(num));
     updatedegree();
     rotateValue=headingDegrees-abs(num);
     currentVal=headingDegrees;
     //this rotate value can have a value either Posetive or Negative
     //Negative indicates less than 360 degree and posetive indicates greater than 0 degree

     if(rotateValue < 0){

       rotateValue=360 - abs(rotateValue);
     }
     flagRotate=1;
     RSSendCommand(RSTurnLeft);
         Serial.println(num);

   }
 else if(String(inData[0])=="z")//This could be any motor start string we choose from the python script
   {
     String in=inData;
     int num=in.substring(2).toInt();
     updatedegree();
     Serial.println("Here is The Value-z");
     Serial.println(String(num));
     rotateValue=(abs(num)+headingDegrees)%360;
     currentVal=headingDegrees;
     flagRotate=-1;
     RSSendCommand(RSTurnRight);
         Serial.println(num);

   }

  else if(String(inData[0])=="d")
  {
    postilt-=5;
    if(postilt < 0){
      postilt = 0;
    }
    tilt.write(postilt);
    delay(30);
  }
  else if(String(inData[0])=="u")
  {
    postilt+=5;
    if(postilt > 90)
    {
      postilt = 90;
    }
    tilt.write(postilt);
    delay(30);
  }
  else if(String(inData[0])=="l")
  {
    pospan+=15;
    if(pospan > 180){
      RSSendCommand(stepLeft);
      pospan=180;
      //inData[0]='\0';
      }
    pan.write(pospan);
    delay(30);
    Serial.println(pospan);
  }
  else if(String(inData[0])=="r")
  {
    pospan-=15;
    if(pospan < 0){
      RSSendCommand(stepright);
      pospan=0;
      //inData[0]='\0';

    }
    pan.write(pospan);
    delay(30);
    Serial.println(pospan);
  }


  else if(String(inData[0])=="L")
  {
    //step left
    RSSendCommand(stepLeft);

  }
  else if(String(inData[0])=="R")
  {
    //step right
    RSSendCommand(stepright);

  }
  // Navigate the ball
  else if(String(inData[0])== "n")
    {
      if(countSearch >= 30){
        countSearch = 0;
        RSSendCommand(stepright);
        delay(2700);
        RSSendCommand(stepright);
        delay(2700);
        RSSendCommand(stepright);
        delay(2700);
        inDataFo=1;
      }
      else{
        if(pospan<120 && pancond1==0)
         {
          pancond2=1;
         }
         if(pancond2==1)
         {
           pospan +=30;
           pan.write(pospan);
         }
         if(pospan>150)
         {
           pancond2=0;
           pancond1=1;
           tiltcond=1;
         }
         if(pancond1==1)
         {
           pospan -=30;
           pan.write(pospan);
         }
         if(pospan<50)
         {
           pospan=30;
           pancond1=0;
         }
         if (pancond1==0 && pancond2==0)
         {
           if(postilt<90 && postilt>=25)
           {
             postilt +=30;
           }
           else if(postilt>=90)
           {
            postilt=25;
           }

         }
        if(postilt<25)
          postilt=25;
        pan.write(pospan);
        delay(30);
        tilt.write(postilt);
        delay(30);
        Serial.println("------+");
        //Serial.println(pospan);
        //Serial.println(postilt);
        delay(100);
    }
    }

    else if(String(inData[0])== "N")
      {
        if(pospan>=150)
        {pospan=30;}
        postilt=90;

        tilt.write(postilt);

        pan.write(pospan);
        delay(30);
        pospan +=30;






      }




  else if(String(inData[0])=="G") //sending data to python
  {
    updatedegree();
    //pospan = pan.read();
    //postilt = tilt.read();
    String data=String(headingDegrees)+"&"+String(pospan)+"&"+String(postilt);
    Serial.println(data);
    serial_connection.println(data);

    }

    else if(String(inData[0])=="J") //sending data to python
    {
      updatedegree();
      delay(10);
      Serial.println("sending position data");
      String data=String(headingDegrees)+"&"+String(forward)+"&"+String(backward);
      Serial.println(data);
      serial_connection.println(data);

      }

  /*  else if(String(inData[0])=="a")//adjust to the right side after after tracking the ball
  {
    boolean a=1;
    int count=0;
      int firstVal,change=0;
      String in=inData;
      if(in.length()>3){
        for (int i = 0; i < in.length(); i++)
        {
          if (in.substring(i, i+1) == "&")
          {
              firstVal = in.substring(0, i).toInt();
              change = in.substring(i+1).toInt();
              break;
          }
        }}
      Serial.print("chanege ");Serial.println(change);
      updatedegree();
      delay(10);
      int current=headingDegrees;
      int desired=(current + change)%360;
      Serial.print("current ");Serial.println(current);
      Serial.print("desired ");Serial.println(desired);
      if(change>40)
      {count=10000;}
      else
      {count=500;}

      while (a) {
      boolean b=1;
      RSSendCommand(RSTurnRight);
      delay(200);
        while(b)
        {
            count --;
            updatedegree();
                if(abs(desired-headingDegrees)<10 || count==0)
                {
                    Serial.print("st");Serial.println(headingDegrees);
                    Serial.print("ch");Serial.println(abs(desired-headingDegrees));
                    RSSendCommand(RSStop);
                    delay(100);
                    for(i=0;i<16;i++){
                    inData[i]='\0';
                    b=0;
                    a=0;

                  }
                }
        }

    }
  }
else if(String(inData[0])=="b")
  {
    boolean a=1;
      int firstVal,change=0;
      Serial.println(inData);
      String in=inData;
      if(in.length()>3){
        for (int i = 0; i < in.length(); i++) {
          if (in.substring(i, i+1) == "&") {
            firstVal = in.substring(0, i).toInt();
            change = in.substring(i+1).toInt();
            break;
          }
        }
      }
        Serial.print("chanege ");Serial.println(change);
        updatedegree();
        int current=headingDegrees;
        int desired = (current - change)%360;
        if(desired<0)
        {
          desired=360+desired;
        }
        Serial.print("current ");Serial.println(current);
        Serial.print("desired ");Serial.println(desired);

        if(change>40)
        {count=10000;}
        else
        {count=500;}
        while (a) {
          boolean b=1;
          RSSendCommand(RSStop);
          delay(20);
          RSSendCommand(RSTurnLeft);
          delay(200);
          while(b){
          count--;
          updatedegree();
          delay(10);
          //Serial.println(headingDegrees);
              if(abs(desired-headingDegrees)<10 || count==0){
              Serial.println("st");
              RSSendCommand(RSStop);
              delay(100);
              for(i=0;i<16;i++){
              inData[i]='0';
              b=0;
              a=0;

            }
              }
          }
      }
  }

*/
  else if(String(inData[0])=="M")
  {
    Serial.println("stepforward");
    RSSendCommand(stepForward);
    delay(500);


  }


  else if(String(inData[0])=="f")
  {
    Serial.println("walk forward");
      RSSendCommand(RSWalkForward);
  }
  else if(String(inData[0])=="s")
  {
      RSSendCommand(RSStop);
  }
  else if(String(inData[0])=="o")
  {
    updatedegree();
    Serial.println("rotating 180 degrees" );
    Serial.println(headingDegrees);
    Serial.println(backward);
    if(abs(headingDegrees-backward)<=60 ||abs(headingDegrees-backward)>=300)
    {
      updatedegree();
      int flag=1;
      int temp=headingDegrees;
      Serial.println("3 rotate to the oppenet direction then kick ball 180 degrees");
      for(int j=0;j<2;j++)
      {
        RSSendCommand(stepright);
        Serial.println("step right");
        delay(2700);
      }
      updatedegree();
      if(abs(temp-headingDegrees)<20)
      {
       serial_connection.println("s");
       serial_connection.println("s");
       flag = 0;
      }
     if(flag == 1){
      for(int j=0;j<2;j++)
      {
        RSSendCommand(stepForward);
        Serial.println("step forward");
        delay(2700);
      }
      for(int j=0;j<2;j++)
      {
        RSSendCommand(stepLeft);
        Serial.println("step left");
        delay(2700);
      }
      for(int j=0;j<3;j++)
      {
        RSSendCommand(stepForward);
        Serial.println("step forward");
        delay(2700);
      }
      for(int j=0;j<2;j++)
      {
        RSSendCommand(stepLeft);
        Serial.println("step left");
        delay(2700);
      }
      for(int j=0;j<2;j++)
      {
        RSSendCommand(stepForward);
        Serial.println("step forward");
        delay(2700);
      }
      for(int j=0;j<1;j++)
      {
        RSSendCommand(stepLeft);
        Serial.println("step left");
        delay(2700);
      }

      serial_connection.println("S");
      serial_connection.println("S");
    }
    }

  }
  else if(String(inData[0])=="x")
  {
    RSSendCommand(RSWalkBackward);
  }




  // kick the ball
  else if(String(inData[0])=="k")
   {
       Serial.println("what to do");
       updatedegree();
       Serial.println(headingDegrees);
       //kick forward
        /*if(abs(headingDegrees-forward)<=10||abs(headingDegrees-forward)>=350)
        {
          RSSendCommand(stepForward);
          Serial.println("kick forward");
          delay(2700);
        }*/
        //adjust to right side and kick
/*        if(((10<(headingDegrees-forward))&&((headingDegrees-forward)<60))||((315<(headingDegrees-forward))&&((headingDegrees-forward)<350)))
        {
          Serial.println("1 adjust to right side and kick 45 degrees");
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepright);
            Serial.println("step right");
            delay(2700);
          }
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepForward);
            Serial.println("step forward");
            delay(2700);
          }
          for(int j=0;j<3;j++)
          {
            RSSendCommand(stepLeft);
            Serial.println("step left");
            delay(2700);
          }

        }
        //adjust to left side and kick
        else if(((10<(forward-headingDegrees))&&((forward-headingDegrees)<45))||((315<(forward-headingDegrees))&&((forward-headingDegrees)<350)))
        {
          Serial.println("2 adjust to left side and kick 45 degrees");
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepLeft);
            Serial.println("step left");
            delay(2700);
          }
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepForward);
            Serial.println("step forward");
            delay(2700);
          }
          for(int j=0;j<3;j++)
          {
            RSSendCommand(stepright);
            Serial.println("step right");
            delay(2700);
          }

        }
*/
        //rotate to the oppenet direction then kick ball

        //adjust to the oppent direction
        if(abs(headingDegrees-left)<=30 || abs(headingDegrees-left)>=330)
        {
          updatedegree();
          int flag=1;
          int temp=headingDegrees;
          // tilt a bit according to the oppenet goal angle and kick ball
          Serial.println("4 adjust to the oppent direction 90 degrees");
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepLeft);
            Serial.println("step left");
            delay(2700);
          }
          updatedegree();
          if(abs(temp-headingDegrees)<20)
          {
           serial_connection.println("s");
           flag = 0;
          }
           if(flag == 1){
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepForward);
            Serial.println("step forward");
            delay(2700);
          }
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepright);
            Serial.println("step right");
            delay(2700);
          }
          for(int j=0;j<1;j++)
          {
            RSSendCommand(stepForward);
            Serial.println("step forward");
            delay(2700);
          }
          for(int j=0;j<1;j++)
          {
            RSSendCommand(stepright);
            Serial.println("step right");
            delay(2700);
          }
          serial_connection.println("S");
          serial_connection.println("S");
        }


        }
        //adjust to the oppenet direction
        else if(abs(headingDegrees-right)<=30 || abs(headingDegrees-right)>=330)
        {
          updatedegree();
          int flag=1;
          int temp=headingDegrees;
          Serial.println("5 adjust to the oppenet direction");
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepright);
            Serial.println("step right");
            delay(2700);
          }
          updatedegree();
          if(abs(temp-headingDegrees)<20)
          {
           serial_connection.println("s");
           serial_connection.println("s");
           flag = 0;
          }
          if(flag == 1){
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepForward);
            Serial.println("step forward");
            delay(2700);
          }
          for(int j=0;j<2;j++)
          {
            RSSendCommand(stepLeft);
            Serial.println("step left");
            delay(2700);
          }
          for(int j=0;j<1;j++)
          {
            RSSendCommand(stepForward);
            Serial.println("step forward");
            delay(2700);
          }
          for(int j=0;j<1;j++)
          {
            RSSendCommand(stepLeft);
            Serial.println("step left");
            delay(2700);
          }
          serial_connection.println("S");
          serial_connection.println("S");

        }

      }
        RSSendCommand(RSStop);
        delay(100);
      }


for(i=0;i<remaining_bytes;i++)//This burns off any remaining bytes that the buffer can't handle.
{
//inChar=serial_connection.read();
}
//Serial.println(inData);//Print to the monitor what was detected
count++;//Increment the line counter
}
//else
//Serial.println("errr");
}
