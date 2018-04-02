// Define pins to be used. On Arduino "D5, D6, D7, D7"
const int pwPin1 = 5;
const int pwPin2 = 6;
const int pwPin3 = 7;
const int pwPin4 = 8;

//variables to store values
long pulse1, pulse2, pulse3, pulse4, cm1, cm2, cm3, cm4;

void setup()
{
  Serial.begin(9600); // Begins serial and defines baud rate. Make sure baud is same in Python
  
  pinMode(pwPin1, INPUT); // Sets each pin as input only - arduino doesn't write out
  pinMode(pwPin2, INPUT);
  pinMode(pwPin3, INPUT);
  pinMode(pwPin4, INPUT);
}

void loop()
{

  //Now reading the physical pwm values
  pulse1 = pulseIn(pwPin1, HIGH);
  pulse2 = pulseIn(pwPin2, HIGH);
  pulse3 = pulseIn(pwPin3, HIGH);
  pulse4 = pulseIn(pwPin4, HIGH);

  //147uS per inch for Maxbotic

  cm1 = pulse1 / 147 * 2.54; // Converts pulse width to cm
  cm2 = pulse2 / 147 * 2.54;
  cm3 = pulse3 / 147 * 2.54;
  cm4 = pulse4 / 147 * 2.54;
 
  Serial.print("#"); //Start symbol
  
  Serial.print(cm1, DEC); // Pin #5
  Serial.print(".");
  
  Serial.print(cm2, DEC); // Pin #6
  Serial.print(".");
  
  Serial.print(cm3, DEC); // Pin #7
  Serial.print(".");
  
  Serial.print(cm4); // Pin #8
  
  Serial.print("$");  //End symbol
    
  Serial.println(); // Important to have new line since Python reads by line
  delay(100);

}

//comment
