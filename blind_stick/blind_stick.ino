
#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "mpr121.h"
#include <Wire.h>
#include "SparkFunLSM6DS3.h"
#include "SPI.h"
#define Serial SerialUSB
#define serialSIM868 Serial1

TinyGsm modem(serialSIM868);
TinyGsmClient client(modem);
PubSubClient makestroClient(client);
LSM6DS3 myIMU;

char response[100];
char latitude[100];
char longitude[100];
float longi;
float lati;

uint8_t a=0;
uint8_t b=0;

const char apn[]  = "internet";
const char user[] = "";
const char pass[] = "";

const char* mqtt_server = "cloud.makestro.com";
const char* topic = "Hisyam_Kamil/BlindStick/data";
const char* sub = "Hisyam_Kamil/BlindStick/control";

int led = 13;
int button = 3;
int baca_button = 0;

long lastMsg = 0;
char msg[50];
int value = 0;
long lastReconnectAttempt = 0;

boolean touchStates[5]; //to keep track of the previous touch states
volatile boolean flagMPR = false;
long debounceDelay = 50; //mempengaruhi delay lampu menyala dan mati (asalnya 500)
long lastDebounce = 0;

template<typename Value>

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Wire.begin();
  Serial.begin(115200);
  mpr121_setup();
  myIMU.begin();
  serialSIM868.begin(115200);

  delay(3000);

  Serial.println("Initializing modem...");
  modem.restart();

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    while (true);
  }
  Serial.println(" OK");

  Serial.print("Connecting to ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" fail");
    while (true);
  }
  Serial.println(" OK");

  makestroClient.setServer(mqtt_server, 1883);
  makestroClient.setCallback(callback);
  delay(5000);
  getloc();
  Serial.print(response);
  for (int i=16 ; i<26; i++) {
    latitude[a] = response[i];
    a++;
  }
  //lati = atof(latitude);
  for (int i=27 ; i<36; i++) {
    longitude[b] = response[i];
    b++;
  }
  Serial.println(latitude);
  Serial.println(longitude); 
}

void mpr121_setup(void){
  set_register(0x5A, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
  set_register(0x5A, MHD_R, 0x01);
  set_register(0x5A, NHD_R, 0x01);
  set_register(0x5A, NCL_R, 0x00);
  set_register(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  set_register(0x5A, MHD_F, 0x01);
  set_register(0x5A, NHD_F, 0x01);
  set_register(0x5A, NCL_F, 0xFF);
  set_register(0x5A, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
  set_register(0x5A, ELE0_T, TOU_THRESH);
  set_register(0x5A, ELE0_R, REL_THRESH);
 
  set_register(0x5A, ELE1_T, TOU_THRESH);
  set_register(0x5A, ELE1_R, REL_THRESH);
  
  set_register(0x5A, ELE2_T, TOU_THRESH);
  set_register(0x5A, ELE2_R, REL_THRESH);
  
  set_register(0x5A, ELE3_T, TOU_THRESH);
  set_register(0x5A, ELE3_R, REL_THRESH);
  
  set_register(0x5A, ELE4_T, TOU_THRESH);
  set_register(0x5A, ELE4_R, REL_THRESH);
  
  set_register(0x5A, ELE5_T, TOU_THRESH);
  set_register(0x5A, ELE5_R, REL_THRESH);
  
  set_register(0x5A, ELE6_T, TOU_THRESH);
  set_register(0x5A, ELE6_R, REL_THRESH);
  
  set_register(0x5A, ELE7_T, TOU_THRESH);
  set_register(0x5A, ELE7_R, REL_THRESH);
  
  set_register(0x5A, ELE8_T, TOU_THRESH);
  set_register(0x5A, ELE8_R, REL_THRESH);
  
  set_register(0x5A, ELE9_T, TOU_THRESH);
  set_register(0x5A, ELE9_R, REL_THRESH);
  
  set_register(0x5A, ELE10_T, TOU_THRESH);
  set_register(0x5A, ELE10_R, REL_THRESH);
  
  set_register(0x5A, ELE11_T, TOU_THRESH);
  set_register(0x5A, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  set_register(0x5A, FIL_CFG, 0x04);
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
    
  // Section F
  // Enable Auto Config and auto Reconfig
  set_register(0x5A, ELE_CFG, 0x0C);
}

void set_register(int address, unsigned char r, unsigned char v)
{
    Wire.beginTransmission(address);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
} 

void loop() {
static uint16_t counter = 0;    

  if (!makestroClient.connected()) {
    reconnect();
  }
  
  makestroClient.loop();
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    Serial.print("Publish message: ");
    Serial.println(msg);
    publishKeyValue("counter",counter);
    counter++;
   
    publishMap(); 

 //Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);

  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatGyroX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatGyroY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatGyroZ(), 4);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C = ");
  Serial.println(myIMU.readTempC(), 4);
  Serial.print(" Degrees F = ");
  Serial.println(myIMU.readTempF(), 4);
  baca_button = digitalRead(button);    
  Serial.println(baca_button);
  //baca_button=digitalRead(button); //untuk membaca nilai button\

   
  double x=myIMU.readFloatAccelX();  
  double y=myIMU.readFloatAccelY();  
  double z=myIMU.readFloatAccelZ();  
 
     if((millis() - lastDebounce) > debounceDelay){
      Wire.requestFrom(0x5A,2); 
    
      byte LSB = Wire.read();
      byte MSB = Wire.read();
    
      uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states
    //uint16_t touched = LSB;

      for (int i=0; i < 12; i++) {  // Check what electrodes were pressed
      if(touched & (1<<i))
        {      
        if(touchStates[i] == 0) // sensor dipegang, led mati
        {
          //pin i was just touched
          //break;
          digitalWrite(led,LOW);
        }
       
      }
      else
      {
        if(touchStates[i] == 1 && x>0.5 && y<0.10) // sensor tdk dipegang dan tongkat jatuh, led menyala (posisi jatuh)
        {
        digitalWrite(led,HIGH);
          //pin released
        }
        else if(touchStates[i] == 1 && x>0.5 && z<0.4) // sensor tdk dipegang dan tongkat jatuh, led menyala (posisi jatuh)
        {
        digitalWrite(led,HIGH);
          //pin released
        }
        else if(touchStates[i] == 1 && x<0.5) // sensor tdk dipegang dan tongkat miring, led menyala (posisi miring)
        {
        digitalWrite(led,LOW);
          //pin released
        }
        
        else if(touchStates[i] == 1 && x<0.05) //sensor tdk dipegang dan tongkat tdk jatuh, led mati
        {
        digitalWrite(led,LOW);
          //pin released
        }
        
      //  touchStates[i] = 0;
      }
     } 
     }

 if (baca_button == 1){
panicButton();
}
}
}

void reconnect() {
  // Loop until we're reconnected
  while (!makestroClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (makestroClient.connect("Hisyam_Kamil-BlindStick-default","Hisyam_Kamil","CjIzfO689VdBxb0O5krsgLk7zdQWEDlwhz49eA0AaZ7b6rW2ZMAryQiguDIqeyE0")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(makestroClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishKeyValue(const char* key, char Valueval) {
    const int bufferSize = JSON_OBJECT_SIZE(2);
    StaticJsonBuffer<bufferSize> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root[key] =  Valueval;

    String jsonString;
    root.printTo(jsonString);
    publishData(jsonString);
  }

void publishData(String payload) {
   publish(topic, payload);
}

void publish(String topic, String payload) {
makestroClient.publish(topic.c_str(), payload.c_str());
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(led, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(led, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

bool getloc()
{
    cleanBuffer();
    serialSIM868.println("AT+CIPGSMLOC=1,1");
    if ( waitFor("OK", "ERROR") != 1 ) return false;


    return true;
}

void cleanBuffer()
{
    delay( 250 );
    while ( serialSIM868.available() > 0) 
    {
        serialSIM868.read();    // Clean the input buffer
        delay(50);
    }
}

int8_t waitFor(const char* expected_answer1, const char* expected_answer2)
{
    uint8_t x=0, answer=0;
    
    unsigned long previous;

    memset(response, (char)0, 100);    // Initialize the string

    delay( 250 );

    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        // if there are data in the UART input buffer, reads it and checks for the asnwer
        if(serialSIM868.available() > 0){
            response[x] = serialSIM868.read();
            x++;
            
            // check if the desired answer 1  is in the response of the module
            if (strstr(response, expected_answer1) != NULL)
            {
                answer = 1;
                Serial.print(response);
                 
            }
            // check if the desired answer 2 is in the response of the module
            else if (strstr(response, expected_answer2) != NULL)
            {
                answer = 2;
                Serial.print(response); 
            }
        }
        delay(10);
        
    }
    // Waits for the asnwer with time out
    while((answer == 0) && ((millis() - previous) < 10000 ));

    return answer;   
}

void publishMap() { 
    
 const int bufferSize = JSON_OBJECT_SIZE(20);
  StaticJsonBuffer<bufferSize> jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();
  root["latitude"] = latitude;
  root["longitude"] = longitude;  

  String jsonStr;
  root.printTo(jsonStr);
  Serial.println(jsonStr);

  publishData(jsonStr);
}

void panicButton() {
/*  long buttonTimer=0;
  long longPressTime = 250;
  boolean longPressActive = false;
  boolean buttonActive = false;
*/  
  const String number = "081214319132";

  /*
  if (digitalRead(button) == HIGH) {
    modem.sendSMS(number,"TC");
    // modem.sendSMS(number,"http://cloud.makestro.com/dashboard/Hisyam_Kamil/BbprHzymsxhrTUvH8NgyAzpYH4SgeOZt/Hisyam_Kamil-BlindStick-default" );
    */
     Serial.println("send sms");
 Serial.println(baca_button);

}

//}
