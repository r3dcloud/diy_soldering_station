#include "Arduino.h" //platformio for visual studio code 
#include <Wire.h>  
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>

#define I2C_ADDR 0x27 //i2c lcd address
#define BACKLIGHT_PIN     3 //lcd config
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

#define MAXTEMP 350 //maximum temperature
#define HEATER 12 //heater pin
#define POT A1 //pot pin
#define ROWS 20 //rows and lines
#define LINES 4

#define DO 6 //max6675 config
#define CS 7
#define CLK 8

float temp_read = 0.0;
float set_temp = 0;
float pid_error = 0;
float prev_error = 0;
float dt, Time, timeprev;
int pid_value = 0;

/*  PID control stands for Proportional, Integral, Derivative 
||  Control which helps us approximate a specific value (in this case temperature)
||  as close as possible. The farther away we are from the setpoint then the 
||  proportional value takes over, but the closest we are the integral value and 
||  the derivative help us approximate the setpoint oscilating slightly above 
||  and below. 
*/  

//PID constants
int kp = 8.1;   
int ki = 0.33;   
int kd = 1.86;

int p_value = 0;    //proportional value based on the error (temp - read_temp)
int i_value = 0;    //integral value 
int d_value = 0;    //derivative value kd * d(error)/dt

MAX6675 thermocouple(CLK, CS, DO); //the thermocouple amplifier object
LiquidCrystal_I2C   lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin); //the lcd object

void print_temp(); //somewhere there's a print_temp function

void setup()
{
  pinMode(HEATER, OUTPUT); //heater as output Note: pwm pin
  Serial.begin(9600); //initialize the serial
  lcd.begin(ROWS, LINES); //initialize the lcd object
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
  lcd.clear();
  Time = millis(); //initialize the millis function
  delay(500); //give them time
}

void loop() {
  set_temp = analogRead(POT); //read the pot value 
  set_temp = map(set_temp, 0,1023,100,MAXTEMP); //convert it to deg C
  temp_read = thermocouple.readCelsius(); //read the thermocouple temp

  pid_error = set_temp - temp_read; //calculate the error
  p_value = kp * pid_error; //the proportional value is the constant * error;
  
  if((-3 < pid_error) || (pid_error<3));
  {
    i_value = i_value + (ki * pid_error); //the integral value is the constant * error, for a small window 
  }

  timeprev = Time;                            //previous time 
  Time = millis();                            //actual time  
  dt = (Time - timeprev) / 1000;     //now we can calculate the dt
  d_value = kd*((pid_error - prev_error)/dt);
  
  pid_value = p_value + i_value + d_value; //final pid_value is the sum of all three values
  
  delay(100);

  if(pid_value < 0)
  {    
    pid_value = 0;    
  }                             //if pid_value is > than 255 then we max it anyways same goes for < 0
  if(pid_value > 255)
  {    
    pid_value = 255;  
  }
  
  delay(200); //delay for the max6675 and the lcd 
  analogWrite(HEATER,pid_value); //output a pwm signal to the MOSFET with a duty cycle 0(full off) - 255(full on)
  prev_error = pid_error;    //store the error for next iteration 
  print_temp(); //print temps and duty cycle
}


void print_temp()
{
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(temp_read);
  lcd.print(" C");
  lcd.setCursor(0,2);
  lcd.print("Duty Cycle: ");
  lcd.print(map(pid_value, 0,255,0,100));
  lcd.print('%');
  lcd.setCursor(0,1);
  lcd.print("Set Temp: ");
  lcd.print(set_temp);
  lcd.print(" C");
}