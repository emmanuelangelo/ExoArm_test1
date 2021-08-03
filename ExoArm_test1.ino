#include <ArduinoTimer.h>
#include <CircularBuffer.h>
#include <CommandHandler.h>
// MEGUNOLINKpro LIBRARIES used for debugging

#include <CommandProcessor.h>
#include <DataStore.h>
#include <DeviceAddress.h>
#include <EEPROMStore.h>
#include <Filter.h>
#include <MegunoLink.h>
#include <MessageHeaders.h>


#include "MegunoLink.h" // Helpful functions for communicating with MegunoLink Pro.
#include "filter.h"
#include "Servo.h"
#include "HX711.h"


HX711 scale(A1, A0); // DOUT, SCK
TimePlot plot1;
TimePlot plot2;
TimePlot plot3;
TimePlot plot4;

int potVal;
int angle;
float dtrate;
double last;



//ExponentialFilter<float> FilteredMuscleValue(5, 20); // NOT USED

void moveup(float writemotor) {
  writemotor = constrain(writemotor, 0 , 255);
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);
  analogWrite(5, writemotor);
  //plot2.SendData("writemotor", writemotor);


}

void stopmove() {
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  analogWrite(5, 0);
}

void movedown(float writemotor) {
  writemotor = constrain(writemotor, 0 , 255);
  digitalWrite(3, LOW);
  digitalWrite(4, HIGH);
  analogWrite(5, writemotor);
  plot2.SendData("writemotor", writemotor);

}


float errorSum;
float Kp = 10;
float Ki = 0.3;

float computePID(float setpoint) {

  float angle1 = analogRead(4);
  angle1 = map(angle1, 0, 424, 145, 55);// dejanski kot

  double time_elapsed = micros();
  dtrate = (time_elapsed - last) / 1000.f;
  last = time_elapsed;

  float error = setpoint - angle1;
  errorSum += error * dtrate;
  errorSum = constrain(errorSum, -255, 255);
  //Serial.println(error);

  //plot3.SendData("Kp", Kp * error);

  return (Kp * error) + (Ki * errorSum);




  //  Serial.println(time_elapsed, 5);
}

int Step = 0;
float upperlimit = 1.64, lowerlimit = -2.0; // between these values is "load free" zone.
int weighton = 0;
float anglespeed;
float writemotor;
float desired_angle_change;
float desired_angle;

int previousMillis = 0;
int interval = 50;
float average_weight;
void setup()
{

  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  scale.set_scale(); //// this value is obtained by calibrating the scale with known weights
  scale.tare();
  angle = analogRead(4);
  angle = map(angle, 0, 424, 145, 55); // angle between forearm and upper arm. ON THE STAND

}



float torque;
int data;
float M  = 45; 

void loop()
{

  int program_runtime = millis();

  float weight = scale.get_units();
  weight = ((weight * 500) / 51060) / 1000; // read the weight on the end of the exoarms' forearm in KG

  //  if (weight <= upperlimit && weight >= lowerlimit) weighton = 0; //check if there is a load on the arm
  //  if (weight > upperlimit) weighton = 1;

  angle = analogRead(4);
  angle = map(angle, 0, 424, 145, 55); // angle between forearm and upper arm. ON THE STAND


  torque = sin(angle / 57.29578) * 0.20 * 9.81 * weight; //

   writemotor = map(torque, -3, 3, -255, 255);;
  
 // writemotor = computePID(M);

  if (writemotor > 5) movedown(writemotor);
  if (writemotor < 5) moveup(-1*writemotor);
  if (writemotor >= 5 && writemotor <= 5) stopmove();

  plot1.SendData("writemotor", writemotor);
  plot3.SendData("angle", angle);
  plot4.SendData("M", M);

}
