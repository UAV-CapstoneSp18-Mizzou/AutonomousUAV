#include <MegunoLink.h>
// Define pins to be used. On Arduino "D9, D10, D11, D12"
const int pwPin1 = 9; //North
const int pwPin2 = 10; //South
const int pwPin3 = 11;  //East
const int pwPin4 = 12; //West
//variables to store values
long pulse1, pulse2, pulse3, pulse4, cm1, cm2, cm3, cm4, cm_1, cm_2, cm_3, cm_4;
void setup()
{
  //This opens up a serial connection to shoot the results back to the PC console
  Serial.begin(9600);
}

void loop()
{
  pinMode(pwPin1, INPUT);
  pinMode(pwPin2, INPUT);
  pinMode(pwPin3, INPUT);
  pinMode(pwPin4, INPUT);
  //Used to read in the pulse that is being sent by the MaxSonar device
  //Pulse Width respresentation with a scale factor of 147 uS per Inch
  pulse1 = pulseIn(pwPin1, HIGH);
  pulse2 = pulseIn(pwPin2, HIGH);
  pulse3 = pulseIn(pwPin3, HIGH);
  pulse4 = pulseIn(pwPin4, HIGH);

  cm1 = pulse1 / 147 * 2.54; //converts pulse width to cm
  cm2 = pulse2 / 147 * 2.54;
  cm3 = pulse3 / 147 * 2.54;
  cm4 = pulse4 / 147 * 2.54;

  cm_1 = 0.8529*cm_1+0.1471*cm1; //filtered pulse
  cm_2 = 0.8529*cm_2+0.1471*cm2;
  cm_3 = 0.8529*cm_3+0.1471*cm3;
  cm_4 = 0.8529*cm_4+0.1471*cm4;

  Serial.print("#");
  Serial.print(cm_1, DEC); // Pin #9
  Serial.print(".");
  Serial.print(cm_2, DEC); // Pin #10
  Serial.print(".");
  Serial.print(cm_3, DEC); // Pin #11
  Serial.print(".");
  Serial.print(cm_4, DEC); // Pin #12
  Serial.print("$");
  Serial.println();
//  TimePlot Plot;
//  Plot.SendData("Filtered cm1", cm_1);
//  //Plot.SendData("Raw1", cm1);
//  Plot.SendData("Filtered cm2", cm_2);
//  //Plot.SendData("Raw2", cm2);
//  Plot.SendData("Filtered cm3", cm_3);
//  //Plot.SendData("Raw3", cm3);
//  Plot.SendData("Filtered cm4", cm_4);
//  //Plot.SendData("Raw4", cm4);

  delay(500); //100ms delay
}
