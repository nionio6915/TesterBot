//
// TesterBot Program Ver 1.0
// License: Attribution-ShareAlike CC BY-SA 
// Originally created by Michael Graham, www.EngineerDog.com
// Remixed by sinned6915
//
// This program operates the complete Testrbot assembly including loadcell feedback,
//  stepper motor control, LCD Readout, & button control
//
//	library dependencies 
//  https://www.airspayce.com/mikem/arduino/AccelStepper/ 
//  https://github.com/dzindra/LCDKeypad
//
// The Circuit: 
//  A0 = is button direction input
//  A1 = home limit switch input, (Switch is normally closed returning 3.3v or 676 bits)
//  A2 = load cell input
//  A3 = top Limit Switch input, (Switch is normally closed returning 3.3v or 676 bits)
//  D4-D10 = LCD screen input
//  D11,12 = step & direction output
//
// Button Commands:
// up key    = MOVE UP WHILE KEY IS HELD
// down key  = MOVE DOWN WHILE KEY IS HELD
// left key  = MOVE TO LOWER HOME POSITION
// right key = STOP MOVING
// select key = AUTO TEST (runs in last direction moved!)
//
//____LCD DECLARATIONS___________________________________________________________________________________________
#include <LiquidCrystal.h>
#include <LCDKeypad.h>
LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7); //Defined LCD, (digital pins used by LCD)
int adc_key_val[5] ={47, 145, 400, 600, 800 }; //(0 = right, 1 = up, 2 = down, 3 = left, 4 = select)
int NUM_KEYS = 5;
int adc_key_in;
int key=-1;
int oldkey=-1;
String Status;

//____STEPPER MOTOR DECLARATIONS_________________________________________________________________________________
#include <AccelStepper.h>
AccelStepper stepper(1, 12, 11);      // Defined Stepper: (# motors, 'step' digital output pin, 'direction' digital output pin)
const int HomeLimitSwitch = 1;        // The analog input pin # for the home switch
const int TopLimitSwitch = 3;         // The analog input pin # for the home switch
const int MoveDisp = 10;              // steps to move if any move button is pressed
const int MaxSpeed = 4000;            // Max Speed in steps per second. 4000 is MAX for Ardu Uno. (See http://www.daycounter.com/Calculators/Stepper-Motor-Calculator.phtml)
const int MaxAcceleration = 4000;     // Steps per second per second
const int StepsPerRevolution = 200;   // Steps per revolution, a physical property of your stepper motor 
float GearRatio = 1.933333;           // Output/Input teeth of gears 29/15
const int RodTPI = 18;                // Threaded rod threads per inch, a 5/16-18 rod is standard.
float HomeLimitSwitchState = 0;       // The current state of the homing limit switch
float TopLimitSwitchState = 0;        // The current state of the homing limit switch

//____LOAD CELL DECLARATIONS______________________________________________________________________________________
const int LoadPin = 2;                // select the input pin for the loadcell
float LoadOffset = 0;                 // Load added to measured load to offset it to zero, lbs
float Load = 0;                       // Sensed load force, lbs.
int OverloadForce = 185;              // Testrbot will shutdown if it detects anything over this force, lbs.<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float Disp = 0;                       // Sensed displacement in inches
const int DelayTime = 200;            // delay X millisec to control acquisition rate: (1000 = 1Hz, 200 = 5 Hz, 20 = 50Hz, 10 = 100Hz)
double LoadScale = 40.00314;          // lb/V, calculated as a function of nominal load, gain, & sensitivity 
double StepPerInch = StepsPerRevolution*RodTPI*GearRatio; //Calculates linear inches of travel per stepper step 
int Direction;                        // This variable will be used to detect the last direction moved

//____LCD SETUP___________________________________________________________________________________________________
 void setup()
{
  Serial.begin(9600);                 // initialize serial communication at 9600 baud. The serial monitor is used for data collection.
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("*TESTRBOT V1.0*");
  lcd.setCursor(0,1);
  lcd.print(" System Reset!");
  delay(2000);

//____STEPPER SETUP________________________________________________________________________________________________
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setAcceleration(MaxAcceleration);
  stepper.setCurrentPosition(0);
  digitalWrite(A1, HIGH);              //This pulls up the analog pin for the home limit switch to clean up the signal. 
  digitalWrite(A3, HIGH);              //This pulls up the analog pin for the home limit switch to clean up the signal. 
  
//____LOADCELL SETUP_______________________________________________________________________________________________
  LoadOffset = -analogRead(LoadPin)*.0048828125*LoadScale;  // Raw loadcell reading * Voltage Input Range divided by bit resolution (5v/1024) * Load cell scale factor. 'LoadOffset' is for setting the 'Load' to zero.
}

//____LOOP__________________________________________________________________________________________________________
void loop()
{ 
  CollectData();  
  delay (DelayTime);
  PrintSerial();
  lcd.clear();
  PrintLCD();
  key = get_key(adc_key_in);            // convert into key press
  if (key != oldkey)                    // if new keypress is detected jump into this large if statement, otherwise restart LOOP
   {
     while (key == 1)                   // _________________________UP key (Move UP if key is held)__________________________________________
     {
       CollectData(); 
       if(TopLimitSwitchState > 1000)   //Make sure you are not at the top before moving up
       {
         Serial.println("REACHED TOP OF TRAVEL, STOPPING!");
         Stop();                        //If you are at the top, stop.
         break;
       }
       if (Load >= OverloadForce)      //If the actuator has hit the overload force STOP.
       {
         Serial.println("REACHED MAX LOAD, STOPPING!");
         Stop();
         break;
       }
       Status = "UP";
       PrintLCD();
       stepper.move(MoveDisp);
       stepper.run(); 
       CollectData(); 
       key = get_key(adc_key_in);     // Convert raw reading into specific key press
       if (key != 1)                  //If you let go of the key, this function exits the loop and restarts LOOP
       {
         oldkey = key;
         Status = " ";
         break;
       }
     }
     while (key == 2)                   //_______________________DOWN key (Move down if key is held)__________________________________________
      {
       CollectData(); 
       if(HomeLimitSwitchState > 1000)   //Make sure you are not at home before moving down
        {
          GoHome();                     //If you are at home its safe to call 'GoHome' Again because it knows to check for the switch status
          break;
        }
        if (Load >= OverloadForce)      //If the actuator has hit the overload force STOP.
         {
           Serial.println("REACHED MAX LOAD, STOPPING!");
           Stop();
           break;
         }
        Status = "DOWN";
        PrintLCD();
        stepper.move(-MoveDisp);
        stepper.run();
        CollectData(); 
        key = get_key(adc_key_in);       // Convert raw reading into specific key press
        if (key != 2)                    // If you let go of the key, this function exits the loop and restarts LOOP
          {
            oldkey = key;
            Status = " ";
            break;
          }
      } 
       if (key == 3)                      //_________________LEFT KEY (Move to home position)______________________________________________
        {
         Status = "HOME";
         PrintLCD();
         PrintSerial();
         GoHome();
        }
        if (key == 4)                     //_______________________SELECT key (AUTO TEST)___________________________________________________
        {
          Status = "TEST";
          PrintLCD();
          PrintSerial();
          TestRun();
        }
   }         
}


//____Function Assignment_____________________________________________________________________________________________

int get_key(unsigned int input)         // This function converts the raw analog signal from A0 to key number
{
    int k;
    for (k = 0; k < NUM_KEYS; k++)
    {
      if (input < adc_key_val[k])
      {
        return k;
      }
    }   
    if (k >= NUM_KEYS)k = -1;            // If no valid key is pressed
    return k;
}

int CollectData()                        //This function collects all available data
{
  Load = abs(analogRead(LoadPin)*.0048828125*LoadScale+LoadOffset+.00001);  // The abs & .00001 prevent the negative sign from showing at 0 lbs.
  Disp = stepper.currentPosition()/StepPerInch;                             // Converts steps into inches
  adc_key_in = analogRead(0);                                               // Check if key is being pressed
  TopLimitSwitchState = analogRead(TopLimitSwitch);                         // Check if top limit switch is tripped
  HomeLimitSwitchState = analogRead(HomeLimitSwitch);                       // Check if home limit switch is tripped
}

int PrintSerial()                        //This function prints all available data to the serial monitor
{
  Serial.print(millis());                // print the time in milliseconds since the program started
  Serial.print(',');                     // print a comma
  Serial.print(Load,6);                  // print the load to serial
  Serial.print(',');                     // print a comma 
  Serial.print(Disp,6);                  // print the displacement to serial
  Serial.print(',');                     // print a comma
  Serial.println(Status);                // print the actuator status to serial (HOME, STOP, UP, DOWN, TEST)
}

int PrintLCD()                           //This function prints all available data to the LCD
{
  lcd.setCursor(0,0); 
  lcd.print("LOAD DISP STATUS");         //Header Line which is always showing
  lcd.setCursor(0,1); 
  lcd.print(Load,1);                     //Current force to 1 decimal place
  lcd.print(" "); 
  lcd.print(Disp);                       //Current displacementr to 2 decimal places
  lcd.print(" "); 
  lcd.print(Status);                     //Current Status (HOME, STOP, UP, DOWN, TEST)
}

int Stop()                               //This function stops all movement
{
  oldkey = key;
  stepper.stop();
  Serial.println("STOPPED ON COMMAND");
  Status = "STOP";
  PrintLCD();
  delay(1000);
}

int GoHome()                             //This function sends the actuator to the bottom home position
{
  Serial.println("GOING HOME");
  Status = "HOME";
  while (true)
 {
   CollectData(); 
   if (HomeLimitSwitchState > 1000)
    {
      oldkey = key;
      stepper.stop();
      Serial.println("AT HOME POSITION");
      Status = "HOME";
      PrintLCD();
      PrintSerial();
      delay(1000);
      break;
    }
    key = get_key(adc_key_in);           // convert into key press
    PrintLCD();
    if (key == 0)                        //0 is the stop key (right button) _________RIGHT KEY______________________________________________________
    {
      Stop();
      break;
    }   
    stepper.move(-MoveDisp);
    stepper.run();    
  }
}

int TestRun()                         //This function sends the actuator in the last direction moved and collects data until hitting a limit switch or reaching the overload force
{
  if (oldkey == 2)                    //If the last direction moved was down, the oldkey will be 2.
  {
    Serial.println("START TENSILE TEST");
    Direction = -1;
  }
  else
  {
    Serial.println("START COMPRESSION TEST");
    Direction = 1;
  }
  while (true)
  {
    CollectData(); 
    if (HomeLimitSwitchState > 1000)   //If the actuator is at the home position I run GoHome which will tell it to stop & note that it is at 'home'
     {
       Serial.println("FINISHED TENSILE TEST RUN");
       GoHome();
       break;
     }
    if (TopLimitSwitchState > 1000)     //If the actuator is at the top position I take it to mean that it has finished a compression run, so I send it home.
     {
       Serial.println("FINISHED COMPRESSION TEST RUN");
       GoHome();
       break;
     }
    if (Load >= OverloadForce)         //If the actuator has hit the overload force I tell it to stop.
     {
       Serial.println("REACHED MAX LOAD, STOPPING!");
       Stop();
       break;
     }     
    key = get_key(adc_key_in);     // convert into key press
    if (key == 0)                  //0 is the right key which means 'stop'
    {
      Stop();
      break;
    }  
    CollectData(); 
    PrintLCD();
    PrintSerial();
    stepper.move(MoveDisp * Direction);
    stepper.run(); 
  }  
}
