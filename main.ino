#include <WiFi.h>

#include "DHT.h"
#include "ThingSpeak.h" 

#define CHANNEL_ID xxx
#define CHANNEL_API_KEY "xxx"

#define WIFI_TIMEOUT_MS 20000
#define DHT11PIN 4
#define moist_sens1 34 
#define moist_sens2 35 
#define moist_sens3 33 
#define relay_pin 5
#include <Servo_ESP32.h>

Servo_ESP32 myservo;
DHT dht(DHT11PIN, DHT11);
WiFiClient  client;

bool pumpstate = false;
int motorposition = 1;
// global variable initialized for resource assignment
// 0 means not assigned
// 1 means assigned to field 1
// 2 means assigned to field 2
// 3 means assigned to field 3

int pump_assigned = 0;
const char ssid[] = "ENTER WIFI NAME";
const char pass[] = "ENTER WIFI PASSWORD";

int moist_read1, moist_read2, moist_read3, moist_disp1, moist_disp2, moist_disp3;
float humi, tempi;

void Thingspeak ( void *pvParameters);
void TempReadandDisplay( void *pvParameters );
void MoistureReadandDisplay1( void *pvParameters );
void MoistureReadandDisplay2( void *pvParameters );
void MoistureReadandDisplay3( void *pvParameters );


void relaycontrol(bool state){
  if(state == true){
    digitalWrite(relay_pin, HIGH);
    Serial.println("Pump on.");
    pumpstate = true;
  }
  else{
    digitalWrite(relay_pin, LOW);
    Serial.println("Pump off.");
    pumpstate = false;
  }
}

void servocontrol(int position){
  switch(position){
    case 1:
      myservo.write(0);
      motorposition = 1;
      Serial.println("Posiiton 0");
      break;
    case 2:
      myservo.write(90);
      motorposition = 2;
      Serial.println("Posiiton 90");
      break;
    case 3:
      myservo.write(180);
      motorposition = 3;
      Serial.println("Posiiton 180");
      break;
    default:
      myservo.write(0);
      motorposition = 1;
      Serial.println("Posiiton 0 using defualt");   
      break;
  }
}


void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void setup() {
pinMode(relay_pin, OUTPUT);
Serial.begin(115200);
myservo.attach(2); 
initWiFi();
Serial.print("RRSI: ");
Serial.println(WiFi.RSSI());
myservo.write(0);
motorposition = 1;
pump_assigned = 0;
dht.begin();

ThingSpeak.begin(client);

xTaskCreate(
 Thingspeak
 , "Thingspeak" // A name just for humans
 , 4096 // This stack size EE-423: Embedded System Design Page 11
 , NULL //Parameters for the task
 , 1 // Priority
 , NULL ); //Task Handle

xTaskCreate(
 TempReadandDisplay
 , "TempRead" // A name just for humans
 , 1000 // This stack size EE-423: Embedded System Design Page 11
 , NULL //Parameters for the task
 , 0 // Priority
 , NULL ); //Task Handle


xTaskCreate(
 MoistureReadandDisplay1
 , "MoistureRead1" // A name just for humans
 , 1000 // This stack size EE-423: Embedded System Design Page 11
 , NULL //Parameters for the task
 , 1 // Priority
 , NULL ); //Task Handle

 xTaskCreate(
 MoistureReadandDisplay2
 , "MoistureRead2" // A name just for humans
 , 1000 // This stack size EE-423: Embedded System Design Page 11
 , NULL //Parameters for the task
 , 1 // Priority
 , NULL ); //Task Handle

 xTaskCreate(
 MoistureReadandDisplay3
 , "MoistureRead3" // A name just for humans
 , 1000 // This stack size EE-423: Embedded System Design Page 11
 , NULL //Parameters for the task
 , 1 // Priority
 , NULL ); //Task Handle
 
}

void loop()
{  
  }

void Thingspeak( void *pvParameters __attribute__((unused)) )
{
for(;;){
  ThingSpeak.setField(1, tempi); //temp
  ThingSpeak.setField(2, humi); //humidity
  ThingSpeak.setField(3, moist_disp1);  //sensor 1
  ThingSpeak.setField(4, moist_disp2); //2
  ThingSpeak.setField(5, moist_disp3); //3
  ThingSpeak.setField(6, pumpstate); //pump status
  ThingSpeak.setField(7, motorposition); //online(always 1)
  
  ThingSpeak.writeFields(CHANNEL_ID, CHANNEL_API_KEY);
  vTaskDelay(pdMS_TO_TICKS(15000));
}
}



void TempReadandDisplay( void *pvParameters __attribute__((unused)) )//Task.
{
 for (;;)
 {
 humi = dht.readHumidity();
 tempi = dht.readTemperature();
 
 Serial.print("Temperature: ");
 Serial.print(tempi);
 Serial.print("ºC ");
 Serial.print("Humidity: ");
 Serial.println(humi);

 vTaskDelay(pdMS_TO_TICKS(2000));
 }
}

void MoistureReadandDisplay1( void *pvParameters __attribute__((unused)) )//Task.
{

 for (;;)
 {
  moist_read1 = analogRead(moist_sens1);
  moist_disp1 = ( 100 - ( (moist_read1/4095.00) * 100 ) );
  Serial.print("Moisture 1 = ");
  Serial.print(moist_disp1);
  Serial.println("%");

  if (pump_assigned == 0 || pump_assigned == 1){
    pump_assigned = 1;
    Serial.println("Entered Critical Section of Sensor 1 Task");
    if(moist_disp1 < 30){
    servocontrol(1);
    relaycontrol(true);
    }
    else{
    relaycontrol(false);
    Serial.println("Leaving Critical Section without using pump.");
    pump_assigned = 0;
  }
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
 }

}

void MoistureReadandDisplay2( void *pvParameters __attribute__((unused)) )//Task.
{

 for (;;)
 {
  moist_read2 = analogRead(moist_sens2);
  moist_disp2 = ( 100 - ( (moist_read2/4095.00) * 100 ) );
  Serial.print("Moisture 2 = ");
  Serial.print(moist_disp2);
  Serial.println("%");
  
  if (pump_assigned == 0 || pump_assigned == 2){
    pump_assigned = 2;
    Serial.println("Entered Critical Section of Sensor 2 Task");
    if(moist_disp2 < 35){
    servocontrol(2);
    relaycontrol(true);
    }
    else{
    relaycontrol(false);
    Serial.println("Leaving Critical Section without using pump.");
    pump_assigned = 0;
  }
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
 }

}

void MoistureReadandDisplay3( void *pvParameters __attribute__((unused)) )//Task.
{
 for (;;)
 {
  moist_read3 = analogRead(moist_sens3);
  moist_disp3 = ( 100 - ( (moist_read3/4095.00) * 100 ) );
  Serial.print("Moisture 3 = ");
  Serial.print(moist_disp3);
  Serial.println("%");

  if (pump_assigned == 0 || pump_assigned == 3){
    pump_assigned = 3;
    Serial.println("Entered Critical Section of Sensor 3 Task");
    if(moist_disp3 < 40){
    servocontrol(3);
    relaycontrol(true);
    }
    else{
    relaycontrol(false);
    Serial.println("Leaving Critical Section without using pump.");
    pump_assigned = 0;
  }
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
 }

}
