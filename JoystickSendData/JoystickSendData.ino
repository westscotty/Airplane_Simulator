/*Purpose: To send data from joysticks serially to a C++ program via a string
  Arduino setup: Arduino, Joystick (x1), Potentiometer (x1)
  Author:    William Keller & Weston Scott
  Date Created:      10/12/20
*/

//Intialize variables, constants, and libraries
#include "math.h"
//float x1Position = 0.0;
//float y1Position = 0.0;
float pot_1 = 0.0;
float x2Position = 0.0;
float y2Position = 0.0;
float rads = 45*M_PI/180;


void setup() {
  //Set up baud rate and pins
  Serial.begin(9600); // C++ uses 9600, if this is changed, make sure to change in C++ aswell
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  //pinMode(A3, INPUT);
}


void loop() {
  //Calculate pot & joystick locations
  pot_1 = (analogRead(A2)/1024.0);
  float h = sqrt(pow(rads,2)*2.0);
  x2Position = (analogRead(A0)-512.0)*h/512.0;  //must go between h and -h
  y2Position = (analogRead(A1)-512.0)*h/512.0;
  //rotate points to find new x and y
  float xp = x2Position*cos(rads) + y2Position*sin(rads);
  float yp = -x2Position*sin(rads) + y2Position*cos(rads);
  //if higher than rad make it rad
  if (xp > rads){
    xp = rads; 
  }
  else if (xp < -rads){
    xp = -rads;  
  }
  if (yp > rads){
    yp = rads;  
  }
  else if (yp < -rads){
    yp = -rads;  
  }
  
  //Send data serially
  outgoingData( xp, yp, pot_1);
}

// Function input: positions of the joystick and pot.
//           output: A serial code of those positions in form of string
void outgoingData(float x, float y, float pot) {
  float data[] = {x, y, pot};
  String str ="<";
  for (int i = 0; i < sizeof(data) / sizeof(data[0]); i++) {
    str.concat(data[i]);
    if (i != sizeof(data) / sizeof(data[0])-1){
      str.concat(",");
    }
  }
  str.concat(",>");
  Serial.println(str);
}
