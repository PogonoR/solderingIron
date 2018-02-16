//*********************** header *******************************
#define version "v0.7"
#define date "2018-02-15"
/* changelog:
 *v.07 (2018-02-15)
  -initial PID implementation
 *v.06.1 (2018-02-15)
  -optimized firmware to update serial output or oled screen only when any value changes,
  -additionally moved driving iron power to separate integer to avoid using raw analog input as power when reached max temp and not cooled down below temp comfort window yet.
 *v.06 (2018-02-15)
  -OLED now functional, showing set temperature, power and temp_overshoot bar graphs
  v.05 (2018-02-14)
  -added OLED screen support
  v0.4 (2018-02-13)
  -actually driving temperature
  v0.3 (2018-02-13)
  -added analog to temperature set conversion
  v0.2 (2018-02-12)
  -added bunch of comments, and header info
*/

/************** heating ******************/
#define heating_pin 9                                        // pin for driving heating controller
#define temp_window 10                                       // define temperature comfort window
boolean heating = 0;
int power;
/************** control ******************/
#define pot_in A0                                            // pin to read controller setting
#include <ResponsiveAnalogRead.h>                            // include analog input filtering library
ResponsiveAnalogRead analog_set(pot_in, true);               // create input setting reading object

/************** measure ******************/
#define temp_read_pin A3                                     // pin for reading temperature
#define current_read_pin A2                                  // pin for checking current
ResponsiveAnalogRead analog_temp(temp_read_pin, true);       // create temperature reading object
ResponsiveAnalogRead analog_current(current_read_pin, true); // createcurrent reading object
unsigned long tick = 0;                                      // placeholder to store time millis snapshots
int temp_read;                                               // placeholder for tempearure reading value
float temp;                                                  // placeholder to hold decoded temperature value
int current_read;                                            // placeholder for current reading value
float current;                                               // placeholder to hold decoded current value

/************** display ******************/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);
char buffor[3];



/**************************************************************************************************/
void setup() {
  tick = millis();                                           // initially fill time variable(millis)
  /************** serial ******************/
  Serial.begin(115200);                                      // initialize serial communication
  Serial.print("Firmware version: ");
  Serial.print(version);
  Serial.print(" (");
  Serial.print(date);
  Serial.println(")");
  /************** heating ******************/
  pinMode(heating_pin, OUTPUT);                              // init heater driving pin

  /************** control ******************/
  pinMode(pot_in, INPUT);                                    // init controller setting pin

  /*************** measure *****************/
  pinMode(temp_read_pin, INPUT);                             // init measure pin: for temperature
  pinMode(current_read_pin, INPUT);                          // init measure pin: for current
  
  /*************** display *****************/
  init_OLED();
}

/************************************************************************************************/
void loop() {
  /************** measure *****************/
  if (millis() - tick > 50) {                                // filter this operation every (XX) often
    analogWrite(heating_pin, 0);                             // turn off heating
    delay(1);                                                // wait 5 millis for currents to drop
	if (analog_temp.hasChanged()){
      ananog_temp.update();                                  // perform temperature measurement
      temp_read = analog_temp.getValue();                    // pull in filtered temperature reading
      temp = map(temp_read, 0, 1023, 20, 450);               // decode temp reading to temperature (C)
	}
    tick = millis();                                         // refresh time snapshot
  } else {                                                   // perform action when not taking temp measurements
    if(analog_current.hasChanged();){
	  analog_current.update();                               // perform current measurement
      current_read = analog_current.getValue();              // pull in filtered current reading
      current = current_read * 3;                            // convert measurement into current value (A)
      current /= 1023;
	}
  };
  /************** control ******************/
  //int val = analogRead(pot_in);                            // read controller setting
  if(analog_set.hasChanged()){
    analog_set.update();                                     // get data from analog input
    int val = analog_set.getValue();                         // pull in filtered inpu setting reading
    float perc = val * 10;                                   // translate RAW controller input into % PWR
    perc = perc / 102;
    int temp_set = map(val, 0, 1023, 20, 450);               // translated input to temperature (C)
  /************** heating ******************/
  if ( temp < (temp_set - temp_window)) {                    // when tip temp is way to low, 
    power = 1023;
    heating = 1;                                             //set heating flag on
  }
  if (temp < temp_set && heating) {                          // when temp is within comfort window, but in heating cycle
    power = 1023;
  }
  if (temp >= temp_set) {                                    // when temperature reaches set one
    power = 0;                                               // stop heating
    heating = 0;                                             // remove heating cycle flag -> cooling cycle
  }
  power = map(power, 0, 1023, 0, 255);                       // use controller setting and convert it into heating drvier
  analogWrite(heating_pin, power);                           // send driver setting to controller
  /*************** serial ******************/
  if (analog_set.hasChanged() || analog_current.hasChanged() || analog_temp.hasChanged()) {
    Serial.print("T: "); Serial.print(temp); Serial.print("("); Serial.print(temp_read); Serial.print(")  ");
    Serial.print("I: "); Serial.print(current); Serial.print("("); Serial.print(current_read); Serial.print(")  ");
    Serial.print("T_set: "); Serial.print(temp_set); Serial.print(" C  ");
   Serial.print("H: "); Serial.print(val);
  
  /*************** display *****************/
    String value_s=dtostrf(temp_set,3,0,buffor);             // convert set temparature to string for sisplay
    draw_value(value_s);                                     // send value as string to display
    draw_power(power);                                       // draw power bar
    draw_ontarget(temp, temp_set);                           // draw temperature overshoot bar graph
    Serial.print("    ");
	Serial.print(PID_power(temp, temp_set));
	Serial.print("\n");                                      // finish serial output line
  };
  delay(20);                                                 // wait a bit for the next iteration loop
}

/**************************************************************************************************/

/**************** heating ******************/
long void PID_power(int temp_curr, int temp_set) {
  long Kp = 2009;
  long Ki =   16;
  long Kd = 2048;
  long  powerr; 
  int   temp_h0, temp_h1;
  const byte denominator_p = 11;  
  
  long kp = Kp * (temp_h1 - temp_curr);
  long ki = Ki * (temp_set - temp_curr);
  long kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
  long delta_p = kp + ki + kd;
  powerr += delta_p; 
  temp_h1 = temp_curr;
  long pwr = powerr + (1 << (denominator_p-1));  // prepare the power to delete by denominator, roud the result
  pwr >>= denominator_p;                        // delete by the denominator
  pwr
  return pwr;
}

/****************** display ****************/
void init_OLED(){                                            // OLED initialization void
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false);          // initialize with the I2C addr 0x3C (for the 128x32)
  // Clear the display buffer.
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(20,10);
  display.println("TS_T-12");
  display.display();
  delay(1000);
  display.clearDisplay();
  }
/*
void display_show(int mode, String value_to_display){
  //oled_timed_out=0;
  //reset_timeout();
  //draw_menu(mode);
	draw_value(value_to_display);
}

void draw_menu(int mode){ //mode menu constructor
    //display.clearDisplay();
    display.setTextColor(1,0);
    display.setTextSize(1);
    display.setCursor(0,0);
	display.print("(D)");
    display.println("");
    display.display();
}
  */
void draw_value(String value_to_display){
	display.setTextSize(2.5);
    display.setCursor(20,11);
    display.setTextColor(1,0);
    display.println(value_to_display+ String(char(247)) + "C");
    display.display();
}
void draw_power(int power){
  power=power/8;
  display.fillRect(125,0,3,32-power, BLACK);
  display.fillRect(125,32-power,3,power, WHITE);
  display.display();
}
void draw_ontarget(int value, int target){
  int target_center=75;
  int overshoot = value - target;
  //int spread=target/10;
  int spread_a=min(-target/10*2*target_center/125,-1);
  int spread_b=max(target/10*2*(125-target_center)/125,1);
  //overshoot=constrain(overshoot, -spread*2*target_center/125, spread*2*(125-target_center)/125);
  overshoot=constrain(overshoot, spread_a, spread_b);
  int pos_a=min(target_center+overshoot*target_center/(-spread_a),target_center);
  int pos_b=max(target_center+overshoot*target_center/(-spread_a),target_center);
  display.fillRect(1,1,pos_a,2, BLACK);
  display.fillRect(pos_a+1,1,pos_b,2, WHITE);
  display.fillRect(pos_b+1,1,125,2, BLACK);
  display.drawLine(target_center,0,target_center,4, WHITE);
  display.display();
}
/*
void oled_display_timeout(){
  unsigned long now=millis();
  byte delta;
  if (oled_timed_out==0 ){
  	if ( tick<=0 )  {//timeout time
      display.invertDisplay(false);
  		display.clearDisplay();
      display.display();
      Serial.println(" screen timed out"); 
  		oled_timed_out=1;
  	} else { //decrease tick
      delta=now-last_tick;
      delta=constrain(delta, 1,1000);
      tick = tick - delta;
      last_tick=now;
  	}
  }
}*/
