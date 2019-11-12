/*****************************************************************
XBee_Serial_Passthrough.ino

Set up a software serial port to pass data between an XBee Shield
and the serial monitor.

Hardware Hookup:
  The XBee Shield makes all of the connections you'll need
  between Arduino and XBee. If you have the shield make
  sure the SWITCH IS IN THE "DLINE" POSITION. That will connect
  the XBee's DOUT and DIN pins to Arduino pins 2 and 3.

*****************************************************************/
// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
//For Atmega328P's
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
//SoftwareSerial XBee(2, 3); // RX, TX

//For Atmega2560, ATmega32U4, etc.
// XBee's DOUT (TX) is connected to pin 10 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 11 (Arduino's Software TX)
SoftwareSerial XBee(10, 11); // RX, TX
#define enA 3
#define in1 4
#define in2 5
#define enB 8
#define in3 6
#define in4 7
#define trig 36
#define echo 34
int motorSpeedA = 0;
int motorSpeedB = 0;
float lev=0;
int rev=0;
long duration;
int distance;

void setup() {
  XBee.begin(9600);
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
//  digitalWrite(enA,HIGH);
//  digitalWrite(enB,HIGH);
  motorSpeedA=0;
  motorSpeedB=0;
  lev=0;
  rev=0;
}

void loop()
{
//  digitalWrite(in1,HIGH);
//  digitalWrite(in2,LOW);
//  digitalWrite(in3,HIGH);
//  digitalWrite(in4,LOW);
//  digitalWrite(enA,HIGH);
//  digitalWrite(enB,HIGH);
//    analogWrite(enA,255);
//    analogWrite(enB,255);
 // delay(5000);
//  digitalWrite(in1,LOW);
//  digitalWrite(in2,HIGH);
//  digitalWrite(in3,LOW);
//  digitalWrite(in4,HIGH);
//  delay(5000);
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    // Sets the trig on HIGH state for 10 micro seconds
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    // Reads the echo, returns the sound wave travel time in microseconds
    duration = pulseIn(echo, HIGH);
    // Calculating the distance
    distance= duration*0.034/2;
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.println(distance);

    if(XBee.available())
    { 
        if(distance < 10)
        {
          XBee.write('9');  
        }
        else
        {
          XBee.write('8');
        }
        char c= XBee.read();
        while(c!='0' && c!='1' && c!='2')
        {
          if(XBee.available())
          {
            c=XBee.read(); 
          }
        }
        while(!XBee.available())
        {
          
        }
        char d= XBee.read();
        while(!XBee.available())
        {
          
        }
        char e=XBee.read();
        while(!XBee.available())
        {
          
        }
        char f=XBee.read();
        
        Serial.write('C');
        Serial.write(c);
        Serial.write('D');
        Serial.write(d);
        Serial.write('E');
        Serial.write(e);
        Serial.write('F');
        Serial.write(f);

        if(f=='m')
        {
          if(lev==0 || (lev!=0 && rev==1))
          {
              rev=1;
              digitalWrite(in1,LOW);
              digitalWrite(in2,HIGH);
              digitalWrite(in3,LOW);
              digitalWrite(in4,HIGH);      
          }
          else
          {
            rev=0;
            digitalWrite(in1,HIGH);
            digitalWrite(in2,LOW);
            digitalWrite(in3,HIGH);
            digitalWrite(in4,LOW); 
          }
        }
        else
        {
          rev=0;
          digitalWrite(in1,HIGH);
          digitalWrite(in2,LOW);
          digitalWrite(in3,HIGH);
          digitalWrite(in4,LOW); 
        }
        if(c=='0' && d=='a')
        {
          //fade
          if(lev>0.2)
          {
            lev-=0.2;
            delay(30);
          }
          else
          {
            lev=0;
          }
         
        }
        else
        {
          if( c=='0')
          {
            lev=0;
          }
          else if(c=='1')
          {
            lev=1;
          }
          else if(c=='2')
          {
            lev=2;
          }
          
          if(d=='a')
          {
            
          }
          else if(d=='b')
          {
            lev--;
          }
          else if(d=='c')
          {
            lev-=2;
          }

          if(lev<0)
          {
            lev=0;
          }
        }

        if(e=='p')
        {
          motorSpeedA= lev * 127;
          motorSpeedB = lev * 127;
        }
        else if(e=='q')
        {
          // decrease B
          motorSpeedA= lev * 127;
          motorSpeedB = (lev-1) * 127;
        }
        else if(e=='r')
        {
          // decrease A
//          motorSpeedB= lev * 127;
//          motorSpeedA = (lev-1) * 127;
          if(lev>0){
            motorSpeedA=0;
          }
          motorSpeedB=lev*127;
        }

        if(motorSpeedA<0)
        {
          motorSpeedA=0;
        }

        if(motorSpeedB<0)
        {
          motorSpeedB=0;
        }
        analogWrite(enA,motorSpeedA);
        analogWrite(enB,motorSpeedB);       
    } 
}
