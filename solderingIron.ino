//*********************** header *******************************
#define version "v0.3"
#define date "2018-02-12"
/* changelog:
v0.3 (2018-02-13)
-added analog to temperature set conversion
v0.2 (2018-02-12)
-added bunch of comments, and header info
*/

/************** heating ******************/
#define heating_pin 9										// pin for driving heating controller
/************** control ******************/
#define pot_in A0											// pin to read controller setting

/************** measure *****************/
#define temp_read_pin A5									// pin for reading temperature
#define current_read_pin A4									// pin for checking current
#include <ResponsiveAnalogRead.h>							// include analog input filtering library
ResponsiveAnalogRead analog_temp(temp_read_pin, true);		// create temperature reading object
ResponsiveAnalogRead analog_current(current_read_pin, true);// createcurrent reading object
unsigned long tick=0;										// placeholder to store time millis snapshots
int temp_read;												// placeholder for tempearure reading value
float temp; 												// placeholder to hold decoded temperature value
int current_read;											// placeholder for current reading value
float current;												// placeholder to hold decoded current value

/**************************************************/
void setup() {
  tick=millis();											// initially fill time variable(millis) 
  /************** serial ******************/
  Serial.begin(115200);										// initialize serial communication
          Serial.print("Firmware version: "); 
          Serial.print(version);
          Serial.print(" ("); 
          Serial.print(date); 
          Serial.println(")"); 
  /************** heating ******************/
  pinMode(heating_pin, OUTPUT);								// init heater driving pin

  /************** control ******************/
  pinMode(pot_in, INPUT);									// init controller setting pin

  /************** measure *****************/
  pinMode(temp_read_pin, INPUT);							// init measure pin: for temperature
  pinMode(current_read_pin, INPUT);							// init measure pin: for current
}
/**************************************************/
void loop() {

  /************** measure *****************/
  //int temp=analogRead(temp_read_pin);
  
  if (millis()-tick >50){									// filter this operation every (XX) often
    analogWrite(heating_pin,0);								// turn off heating
    delay(5);												// wait 5 millis for currents to drop
    analog_temp.update();									// perform temperature measurement
    temp_read=analog_temp.getValue();						// pull in filtered temperature reading
    temp=map(temp_read,0,1023,20,450);						// decode temp reading to temperature (C)
    tick=millis();											// refresh time snapshot
  }else{													// perform action when not taking temp measurements
   analog_current.update();									// perform current measurement
     current_read=analog_current.getValue();				// pull in filtered current reading
     current=current_read*3;								// convert measurement into current value (A)
     current/=1023;
  };
  /************** serial ******************/
  Serial.print("T: "); Serial.print(temp);Serial.print("("); Serial.print(temp_read);Serial.print(") ");
  Serial.print("I: "); Serial.print(current);Serial.print("("); Serial.print(current_read);Serial.print(") ");
  /************** control ******************/
  int val=analogRead(pot_in);								// read controller setting
  float perc = val*10;										// translate RAW controller input into % PWR
  perc = perc /102;
  /************** serial ******************/
  Serial.print("A_set: "); Serial.print(val);Serial.print(" ("); Serial.print(perc);Serial.print("% )   ");
  /************** heating ******************/
  val=map(val,0,1023,0,255);								// use controller setting and convert it into heating drvier
  analogWrite(heating_pin,val);								// send driver setting to controller
  /************** serial ******************/
  Serial.print("H: "); Serial.print(val);
  Serial.print("\n");
}
/**************************************************/
