
/* Project:RC Car via Bluetooth with Android Smartphone. 
 * Featutes: manual steering and auto-pilot mode. Speed and colision avoidance can be adjusted in App.
 * Inspired by project of Vasilakis Michalis // 12-12-2014 ver.2 test. More information at www.ardumotive.com http://www.ardumotive.com/bluetooth-rc-car.html
 * Backward compatible with Ardumotive App
 * Activ A=left Active B = right
 */
  
  #include <Ultrasonic.h>      // Sketch is built on Raspberry Pi using  https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
  
 // HC-SRO4  TX       = 0;     // to upload sketch you must /!\ disconnect TX from HC-SRO4 TX - disconnect wire from D0
 // HC_SRO4  RX       = 1;     // can be connected during sketch upload
 // not in use        = 2;     // reserved for Bluetooth state usage
  const int buzzer    = 3;     // Buzzer / Speaker to Arduino UNO Pin 3
  const int LEDs      = 4;     // Leds connected to Arduino UNO Pin 12
  const int motorA1   = 5;     // pin 5 supports PMWM IN 1 of L298n mini
  const int motorA2   = 6;     // pin 6 supports PMWM IN 2 of L298n mini
  Ultrasonic ultraright(7,8);  // (Trig PIN,Echo PIN)
  const int motorB1   = 9;     // pin 9 supports PMWM IN 3 of L298n mini
  const int motorB2  = 10;     // pin 10 supports PMWM IN 4 of L298n mini
  Ultrasonic ultraleft(11,12); // (Trig PIN,Echo PIN)
 //not in use      = 13;
  
  int LEDst =0;       // to on and off head lamp. Default state off
  int Hornst=0;       // to on and off Hornst. Default state off
  int RCmess;         // to store income message via Bluetooth
  int LSpeed;         // Left  wheel speed
  int RSpeed;         // Right wheel speed
  int BSpeed;         // Backward    speed
  int RSpeed2;        // Half speed
  int LSpeed2;        // Half speed
  int BSpeed2;        // Half speed
  int Sens = 1;       // Set sensetivity of obstalce avoidance
  int Ldist;          // distance measured by right sensor to manage left wheel speed
  int Rdist;          // distance measured by left sensor to manage right wheel speed
  int Lcorr;          // Speed correction (decrease) when obstacle detected
  int Rcorr;          // Speed correction (decrease) when obstacle detected
  int Gear=3;         // Default Gear value, to let the car move before gear is changed in Android app

void setup() {
   
 // pinMode (buzzer) when using tone function in not required
    pinMode(LEDs,    OUTPUT); 
    pinMode(motorA1, OUTPUT); // initially L298N mini was used, than replaced by custom dual MOSFET H-bridge https://easyeda.com/olegburak.by/h-bridge-pair-wo-opto
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
 // pinMode for ultrasonic sensors are set by ultrasonic.h  
    
    Serial.begin(9600); // Initialize serial communication at 9600 bits per second:
}
 
void loop() {

   if(Serial.available() > 0){     
     RCmess = Serial.read();   // Save income data to variable 'RCmess'
      // Serial.write(RCmess); // for debug: susspend car wheels and keep USB connected, then use Serial monitor to see received command via Bluetooth
      // Serial.println();     // for debug
      }
    
   Ldist = ultraleft.Ranging(CM);          //ultraleft.Ranging(CM) is a distance returned by ultrasonic.h in cm, max value is 51.
   //Serial.print("Ldist: ");              // for debug: susspend car wheels and keep USB connected, then use Serial monitor to see measured distance
   //Serial.print(ultraleft.Ranging(CM));  // CM or INC
   //Serial.print(" cm     " );
   //delay(50);
   Lcorr = (55-Ldist);                     // Lcorr reduce speed of opposite motor. To stop motor 10 cm before obstacle, constanta is 61 
   //Serial.print("Lcorr: ");
   //Serial.print(Lcorr);                  // CM or INC
   //Serial.print(" cm     " );
   //delay(50);
   Rdist = ultraright.Ranging(CM);         // ultraleft.Ranging(CM) is a distance returned by ultrasonic.h in cm,  max value is 51.
   //Serial.print("Rdist: ");
   //Serial.print(ultraright.Ranging(CM)); // CM or INC
   //Serial.print(" cm" );
   //delay(50);
   Rcorr = (55-Rdist);                     // Lcorr reduce speed of opposite motor
   //Serial.print("Rcorr: ");
   //Serial.print(Rcorr); // CM or INC
   //Serial.println(" cm     " );
   //delay(50);

 /***********************Gearbox**********************/   
    if      (RCmess == '0'){ //Change speed if RCmess is equal from 0 to 4. 
      Gear= 1;
      }
    else if (RCmess == '1'){
      Gear= 2;
      }
    else if (RCmess == '2'){
      Gear= 3;
      }
    else if (RCmess == '3'){
      Gear= 4;
      }
    else if (RCmess == '4'){
      Gear= 5;
      }   

 /***********************Sensetivity**********************/   
    if      (RCmess == '5'){   // Change Sensetivity if RCmess is equal from 5 to 9. 
      Sens= 1000;              // "0" in App,minimize correction
      }
    else if (RCmess == '6'){
      Sens = 3;                // "1' in App, reduce correction by 3 times
      }
    else if (RCmess == '7'){
      Sens = 2;                // "2' in App, reduce correction by 2 times
      }
    else if (RCmess == '8'){
      Sens = 1;                // "3' in App, no correction reduction
      }
         
    LSpeed= ((51-(Lcorr/Sens))*Gear);  //50 is a base speed, multiplied by Gear (1-5) produce argument for analog write (50-250). To stop motor 10cm before obstacle Lspeed can be negative if distance will be closer than 10cm
    if (LSpeed < 0) {
       LSpeed = 0;
     }  
    RSpeed= ((51-(Rcorr/Sens))*Gear);
    if (RSpeed < 0) {
       RSpeed = 0;
     }   
    BSpeed= 50*Gear;                  // There is no sensor in back, so there is no correction of backward speed 
  
    LSpeed2=LSpeed/1.7;
    RSpeed2=RSpeed/1.7;
    BSpeed2=RSpeed/1.7;
    
  /***********************Forward****************************/
    if (RCmess == 'F') {         //If RCmess is equal with letter 'F', car will go forward!
    	analogWrite(motorB1, LSpeed); analogWrite(motorA1, RSpeed);
        analogWrite(motorB2, 0);      analogWrite(motorA2, 0); 
    }
  /**********************Forward Left************************/
    else if (RCmess == 'G') {    //If RCmess is equal with letter 'G', car will go forward left
    	analogWrite(motorB1, LSpeed2); analogWrite(motorA1, RSpeed);  
        analogWrite(motorB2, 0);       analogWrite(motorA2, 0); 
    }
  /**********************Forward Right************************/
    else if (RCmess == 'I') {   //If RCmess is equal with letter 'I', car will go forward right
      	analogWrite(motorB1, LSpeed); analogWrite(motorA1, RSpeed2); 
        analogWrite(motorB2, 0);      analogWrite(motorA2, 0); 
    }
  /***********************Backward****************************/
    else if (RCmess == 'B') {  //If RCmess is equal with letter 'B', car will go backward
    	analogWrite(motorB1, 0);      analogWrite(motorA1, 0); 
        analogWrite(motorB2, BSpeed); analogWrite(motorA2, BSpeed); 
    }
  /**********************Backward Left************************/
    else if (RCmess == 'H') {   //If RCmess is equal with letter 'H', car will go backward left
    	analogWrite(motorB1, 0);       analogWrite(motorA1, 0); 
        analogWrite(motorB2, BSpeed2); analogWrite(motorA2, BSpeed); 
    }
  /**********************Backward Right************************/
    else if (RCmess == 'J') {  //If RCmess is equal with letter 'J', car will go backward right
    	analogWrite(motorB1, 0);      analogWrite(motorA1, 0); 
        analogWrite(motorB2, BSpeed); analogWrite(motorA2, BSpeed2); 
    }
  /***************************Left*****************************/
    else if (RCmess == 'L') {  //If RCmess is equal with letter 'L', wheels will turn left
    	analogWrite(motorB1, 0);       analogWrite(motorA1, RSpeed); 
        analogWrite(motorB2, 0);       analogWrite(motorA2, 0); 
    }
  /***************************Right*****************************/
    else if (RCmess == 'R') {  //If RCmess is equal with letter 'R', wheels will turn right
    	analogWrite(motorB1, RSpeed); analogWrite(motorA1, 0); 
        analogWrite(motorB2, 0);      analogWrite(motorA2, 0); 		
    }
  /***************************Rotate clockwise*****************************/
    else if (RCmess == 'Y') {  //If RCmess is equal with letter 'Y', wheels will run in opposite direction - rotate clockwise
    	analogWrite(motorB1, RSpeed); analogWrite(motorA1, 0); 
        analogWrite(motorB2, 0);       analogWrite(motorA2, BSpeed); 
    }
  /***************************Rotate counter clockwise*****************************/
    else if (RCmess == 'Z') {  //If RCmess is equal with letter 'Z', wheels will run in opposite direction - rotate counter clockwise
    	analogWrite(motorB1, 0);      analogWrite(motorA1, RSpeed); 
        analogWrite(motorB2, BSpeed); analogWrite(motorA2, 0); 
    }
  /************************Stop*****************************/
    else if (RCmess == 'S'){    //If RCmess is equal with letter 'S', stop the car
        analogWrite(motorB1, 0);  analogWrite(motorA1, 0); 
        analogWrite(motorB2, 0);  analogWrite(motorA2, 0);
    }
  /************************Auto pilot*****************************/
    else if (RCmess == 'A'){    //If RCmess is equal with letter 'A', car constantly move forward. Sensetivity <> '0' ! Any message disable autopilot
        RCmess='F';
    }
  /************************Head lamps*****************************/
    else if (RCmess == 'W') {  //If RCmess is equal with letter 'W', turn leds on or off
      if      (LEDst == 0){  
         digitalWrite(LEDs, HIGH); 
         LEDst = 1;
      }
      else if (LEDst == 1){
         digitalWrite(LEDs, LOW); 
         LEDst = 0;
      }
      RCmess='n';
    }
  /**********************Horn sound***************************/
    else if (RCmess == 'V'){   //If RCmess is equal with letter 'V', play (or stop) Hornst sound     
      if     (Hornst == 0){  
         tone  (buzzer, 1000); //Speaker on 
         Hornst = 1;
      }
      else if (Hornst ==1){
         noTone(buzzer);      //Speaker off 
         Hornst = 0;
      }
      RCmess='n';  
    }
      /***************************Rotate clockwise*****************************/
    if ((ultraleft.Ranging(CM) < 15) && (ultraright.Ranging(CM) < 15) && RCmess == 'F') {  //If car stuck it  - rotate clockwise
        delay (1000);
    	analogWrite(motorB1, 200); analogWrite(motorA1, 0); 
        analogWrite(motorB2, 0);   analogWrite(motorA2, 200);
        delay (300); 
    }  
}

/* Letters in RCmess
 * 'F' forward
 * 'G' forward left
 * 'I' forward right
 * 'B' backward
 * 'H' backward left
 * 'J' backward right
 * 'L' left
 * 'R' right
 * 'Y' rotate clockwise
 * 'Z' rotate counter clockwise
 * 'S' stop
 * 'W' leds on or of off
 * 'V' Hornst sound on or off
 * 'A' autopilot 
 */
