// Code for the automatisation with the Arduino Uno of the Greenhouse

// Pins outside (Always from top to bottom: Ground - Vcc - Signal)
// 123    Switch with Arduino       MOSFET 1
// 456    Water pump                MOSFET 2 (Only Plus/Minus needed for this pump)
// 789    Switched with water pump  MOSFET 2
// 101112 Servo                     MOSFET 3  (Now internal!)
// 131415 DHT11 - Termperature      MOSFET 3
// 161718 Ground moisture sensor    MOSFET 3
// 192021 Water level sensor        MOSFET 3 value empty:80 value full: 490

#include <EEPROM.h>
#include <Time.h>
#include <Servo.h>
#include "DHT.h"

// Digital pins
#define Mosfet_2_port         2 // Switch for the water pump
#define Mosfet_3_port         3 // Switch for all Sensors, Servo and Stepper motor

#define free_digital_port_1   4 // Power for this sensor/actuator is switched with the Arduino      MOSFET 1
#define free_digital_port_2   5 // The power to this port is used by the water pump                 MOSFET 2
#define free_digital_port_3   6 // Power for this sensor/actuator is switched with the water pump   MOSFET 2
#define servo_port            7 // Control port for Servo Motor                                     MOSFET 3
#define temp_sensor_port      8 // Temperature and humidity sensor                                  MOSEFT 3

// Digital Stepper Motor Pins
#define stepper_port1 9  
#define stepper_port2 10
#define stepper_port3 11
#define stepper_port4 12

// Analog Pins
#define battery_port                A0  // Port to read the battery voltage   
#define ground_moisture_sensor_port A4  // Port for the ground moisture sensor                 MOSEFT 3 
#define water_level_sensor_port     A5  // Port for the water level sensor in the bucket       MOSEFT 3

//Flags for controlling which components run
bool demo_flag = true;
bool servo_flag = true;
bool pump_flag = true;
bool stepper_flag = true;
bool sensor_flag = true;

bool save_battery_flag = false;
bool mem_clear_flag = false;

//Variables
int pump_msecs = 10000; //TODO
int good_moisture_percentage = 100; //TODO
int good_temperature = 20; //TODO
unsigned char all_good_angle = 0;
unsigned char empty_battery_angle = 60;
unsigned char empty_water_angle = 120;
unsigned char empty_battery_water_angle = 180;
int roof_up_rotations = -80; //TODO
int current_rotations = 0;

float battery_voltage = 0.0;
float temperature = 0.0;
float humidity = 0.0;
float ground_moisture_percentage = 0.0;
float water_level = 0.0;

//Stepper variables
#define FULL_ROTATION 4095
const int phases1[] = {0, 0, 0, 0, 0, 1, 1, 1};
const int phases2[] = {0, 0, 0, 1, 1, 1, 0, 0};
const int phases3[] = {0, 1, 1, 1, 0, 0, 0, 0};
const int phases4[] = {1, 1, 0, 0, 0, 0, 0, 1};
int Phase = 0;
int Speed = 100; //MUST BE 1 - 100

//DHT Temperatur Sensor
#define DHTTYPE DHT11
DHT dht(temp_sensor_port, DHTTYPE);

//Servo object
Servo myservo;
int pwm;

//For reading the battery voltage
#define NUM_SAMPLES 15
float min_battery_voltage = 3.0; // vorher 3.4;
int eeprom_addr = 0; // For saving if the roof is up or down

void stepper(int count, bool _stepper_flag)
{
  if(_stepper_flag==true)
  {
    pinMode(stepper_port1, OUTPUT);
    pinMode(stepper_port2, OUTPUT);
    pinMode(stepper_port3, OUTPUT);
    pinMode(stepper_port4, OUTPUT);

    int val = EEPROM.read(eeprom_addr);

    int rotationDirection = count < 1 ? -1 : 1;
    count *= rotationDirection;
    for (int x = 0; x < count; x++)
    {
      digitalWrite(stepper_port1, phases1[Phase]);
      digitalWrite(stepper_port2, phases2[Phase]);
      digitalWrite(stepper_port3, phases3[Phase]);
      digitalWrite(stepper_port4, phases4[Phase]);
      IncrementPhase(rotationDirection);
      delay(100/Speed);
      EEPROM.write(eeprom_addr,val + x*rotationDirection);
    }

    pinMode(stepper_port1, INPUT);
    pinMode(stepper_port2, INPUT);
    pinMode(stepper_port3, INPUT);
    pinMode(stepper_port4, INPUT);
  }
}

void IncrementPhase(int rotationDirection) // TODO understand
{
  Phase += 8;
  Phase += rotationDirection;
  Phase %= 8;
}

float read_battery_voltage(int port) //TODO get right value --> vielleicht keinen Spannungsteiler, sondern direkt Spannung an der Batterie ablesen, da diese nicht über 5V gehen wird
{
  delay(1000);
  //Read current battery voltage
  int sum;                          // sum of samples taken
  unsigned char sample_count;       // current sample number
  float voltage;                    // calculated voltage
  while (sample_count < NUM_SAMPLES) 
  {
    sum += analogRead(port);
    sample_count++;
    delay(100);
  }
  voltage = (((float)sum / (float)NUM_SAMPLES) / 1024.0)*11.38*5.07;
  return (((float)sum / (float)NUM_SAMPLES) / 1024.0); //TODO delete
  return voltage;
}

void pump(int pump_msecs, bool _pump_flag)
{
  if(_pump_flag==true)
  {
    digitalWrite(Mosfet_2_port, HIGH);
    delay(pump_msecs);
    digitalWrite(Mosfet_2_port, LOW);
  }
}

void turn_servo(unsigned char angle, bool _servo_flag)
{
  if(_servo_flag==true)
  {
    myservo.write(angle);
    delay(400);
  }
}

float read_temperature(bool _sensor_flag)
{
  if(_sensor_flag==true)
  {
    delay(500);
    float t = dht.readTemperature();
    return t;
  }
}

float read_humidity(bool _sensor_flag)
{
  if(_sensor_flag==true)
  {
    delay(500);
    float h = dht.readHumidity();
    return h;
  }
}

float read_water_level(int port, bool _sensor_flag)
{
  if(_sensor_flag==true)
  {
    delay(500);
    int value = analogRead(port); //Read data from analog pin and store it to value variable

    //return value; //TODO delete
    if (value<=100){
      return 0.0;
    }
    else
    {
      return 1.0;
    }
//    else if (value>280 && value<=330){
//      return 0.005; 
//    }
//    else if (value>330 && value<=415){ 
//      return 0.01;
//    }
//    else if (value>415 && value<=460){ 
//      return 0.015;
//    } 
//    else if (value>460 && value<=480){ 
//      return 0.02;
//    }
//    else if (value>480 && value<=490){ 
//      return 0.025;
//    }
//    else if (value>490 && value<=500){ 
//      return 0.03;
//    }
//    else if (value>500 && value<=505){ 
//      return 0.035;
//    }
//    else if (value>505){ 
//      return 0.04;
//    }
  }
}

float read_ground_moisture(int port, bool _sensor_flag)
{
  if(_sensor_flag==true)
  {
    delay(500);
    int output_value= analogRead(port);
    //return output_value;
    return map(output_value,550,10,0,100);
  }
}

void print_sensor_values(float v, float t, float h, float w, float m, float r)
{
    Serial.println("##########################");
    Serial.print("Battery voltage: ");
    Serial.print(v);
    Serial.print(" V ");
    Serial.println();

    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %");
    Serial.println();
    
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.print(" *C ");
    Serial.println();

    Serial.print("Water level: ");
    Serial.print(w);
    Serial.print(" m ");
    Serial.println();

    Serial.print("Moisture: ");
    Serial.print(m);
    Serial.print(" %");
    Serial.println();

    Serial.print("Current rotations: ");
    Serial.print(r);
    Serial.println();
    Serial.println("##########################");
}
void blink(int number_of_blinks, int msec)
{
  for (int x = 0; x < number_of_blinks; x++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(msec);
    digitalWrite(LED_BUILTIN, LOW);
    delay(msec);
  }
}

void demo()
{
  digitalWrite(Mosfet_3_port, HIGH);
  
  blink(1,1000);
  battery_voltage = read_battery_voltage(battery_port);
  temperature = read_temperature(sensor_flag);
  humidity = read_humidity(sensor_flag);
  water_level = read_water_level(water_level_sensor_port, sensor_flag);
  ground_moisture_percentage = read_ground_moisture(ground_moisture_sensor_port, sensor_flag);
  current_rotations = EEPROM.read(eeprom_addr);
  print_sensor_values(battery_voltage, temperature, humidity, water_level, ground_moisture_percentage, current_rotations);

  delay(1000);

  blink(1,200);
  turn_servo(all_good_angle, servo_flag);
  turn_servo(empty_battery_angle, servo_flag);
  turn_servo(empty_water_angle, servo_flag);
  turn_servo(empty_battery_water_angle, servo_flag);

  delay(1000);
  
  blink(2,200); 
  stepper(FULL_ROTATION, stepper_flag);

  digitalWrite(Mosfet_3_port, LOW);

  delay(1000);

  blink(3,200);
  pump(pump_msecs, pump_flag);

  delay(1000);
}

void setup() 
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Mosfet_2_port, OUTPUT);
  pinMode(Mosfet_3_port, OUTPUT);
  pinMode(stepper_port1, OUTPUT);
  pinMode(stepper_port2, OUTPUT);
  pinMode(stepper_port3, OUTPUT);
  pinMode(stepper_port4, OUTPUT);
  pinMode(servo_port, OUTPUT);

  pinMode(temp_sensor_port, INPUT);
  pinMode(ground_moisture_sensor_port, INPUT);
  pinMode(water_level_sensor_port, INPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(Mosfet_2_port, LOW);
  digitalWrite(Mosfet_3_port, LOW);

  dht.begin();
  myservo.attach(servo_port);
}

void loop() {

  if(mem_clear_flag == true) // Clear EEPROM
  {
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }
    save_battery_flag = true;
  }

  if(save_battery_flag == false)
  {
    if(demo_flag==true) //Run all Sensors and motors and print the values
    {
      demo();
    }
    else  //Normal program
    {
      blink(1,1000);
      battery_voltage = read_battery_voltage(battery_port);

      if(battery_voltage>min_battery_voltage && !isnan(battery_voltage))
      {
        blink(1,200);
        turn_servo(all_good_angle, servo_flag);
  
        digitalWrite(Mosfet_3_port, HIGH);
        temperature = read_temperature(sensor_flag);
        humidity = read_humidity(sensor_flag);
        ground_moisture_percentage = read_ground_moisture(ground_moisture_sensor_port, sensor_flag);
        water_level = read_water_level(water_level_sensor_port, sensor_flag);
            
        if(water_level==0.0 && !isnan(water_level)) //Only water empty
        {
          turn_servo(empty_water_angle, servo_flag);
        }
        else
        {
          turn_servo(all_good_angle, servo_flag); // Everything is alright
        }

        if(temperature>good_temperature && !isnan(temperature) && temperature > -5 && temperature < 100) // Open roof
        {
          int val = EEPROM.read(eeprom_addr); //TODO 
          

          blink(2,200); 
          stepper(roof_up_rotations*FULL_ROTATION, stepper_flag);
        }
        else // Close roof
        {
          int val = EEPROM.read(eeprom_addr); //TODO delete
           
          blink(2,200); 
          stepper(-roof_up_rotations*FULL_ROTATION, stepper_flag);
        }
        digitalWrite(Mosfet_3_port, LOW);
  
        if(ground_moisture_percentage<good_moisture_percentage && !isnan(ground_moisture_percentage) && ground_moisture_percentage >= 0 && ground_moisture_percentage <= 100)
        {
          pump(pump_msecs, pump_flag);
        }
      } 
      else
      {
        if(water_level>0.0 && !isnan(water_level)) //Only battery empty
        {
          turn_servo(empty_battery_angle, servo_flag);
        }
        else // Battery and water empty
        {
          turn_servo(empty_battery_water_angle, servo_flag);
        }
        digitalWrite(Mosfet_2_port, LOW);
        digitalWrite(Mosfet_3_port, LOW);
        save_battery_flag = true; //Put Microcontroller into Idle and not doing anything to save power
      }
    }
  }
  else
  {
    digitalWrite(Mosfet_2_port, LOW);
    digitalWrite(Mosfet_3_port, LOW);
    delay(180000);
  }
}
