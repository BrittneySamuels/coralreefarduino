#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <HTTPClient.h>
#include <Wire.h>


// REPLACE WITH THE MAC Address of your receiver 
uint8_t hubAddress[] = {0x24, 0x6F, 0x28, 0xB0, 0x82, 0x54};

// Define variables to store sensor readings to be sent
float temperature;
float water;
float pH;
float turbidity;

// Define variables to store incoming user measures
float incomingTemp;
float incomingWater;
float incomingPH;
float incomingTurb;

// Variable to store if sending data was successful
String success;

//Structure example to send data
typedef struct struct_message {
    float temp;
    float wat;
    float ph;
    float turb;
    float Heater;
    float Inpump;
    float Outpump;
} struct_message;


// Create a struct_message called BME280Readings to hold sensor readings
struct_message sendReadings;

// Create a struct_message to hold incoming sensor readings
struct_message userMeasures;


// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&userMeasures, incomingData, sizeof(userMeasures));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingTemp = userMeasures.temp;
  incomingWater = userMeasures.wat;
  incomingPH = userMeasures.ph;
  incomingTurb = userMeasures.turb;
  Serial.println ("Recieved Temperature:");
  Serial.println (incomingTemp);
  Serial.println ("Recieved Water Level:");
  Serial.println (incomingWater);
  Serial.println ("Recieved pH:");
  Serial.println (incomingPH);
  Serial.println ("Recieved TDS:");
  Serial.println (incomingTurb);
}


 //GLOBAL DEFINITIONS AND PORT IDENTIFICATIONS
    //Lights and Sound
    #define heater 23// port for Increase Temp 
  
  //Water Level Sensor
     #define waterSensor 39 //Collects analog reading

    //temperature
    const int oneWireBus = 5;
    OneWire oneWire(oneWireBus);
    DallasTemperature sensors(&oneWire); 
    int highTempCount = 0; //Temp counter for warnings (HIGH)
    int lowTempCount = 0; //Temp counter for warnings (LOW)
    


    //Water Pumps
    #define iFPower 14 //Powers the IN Flow water pump
    #define oFPower 27 //Powers the OUT Flow water pump
    int waterMax = 700; //highest water level
    int waterMin = 500; //lowest water level

//pH sensor
    #define SensorPin 34          // the pH meter Analog output is connected with the Arduino’s Analog
    unsigned long int avgValue;  //Store the average value of the sensor feedback
    float b;
    int buf[10],temp;

//TDS Sensor
    #define sensorPin 2
 
    int sensorValue = 0;
    float tdsValue = 0;
    float Voltage = 0;
    float tds = 0;
    float TDSCon = 0.000998859;
    float TDS = 0; 
    float factor = 33.85;
    float TDSg = 0;
    
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Init ESP-NOW
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, hubAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
   else 
   {
    Serial.println("Peer Added Successfully!");
   }
   
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

//Define OUTPUTS
pinMode (heater, OUTPUT);
pinMode (iFPower, OUTPUT);
pinMode (oFPower, OUTPUT);
pinMode (13,OUTPUT); 

Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}


 
void loop() {

    float Heater = 1000;
    float Inpump = 1000;
    float Outpump = 1000;
    float Blank = 1000;

   // Confirm Connection
    if (esp_now_init() == ESP_OK) 
    {
    Serial.println(" ESP-NOW Connected");
    }
    if (esp_now_init() != ESP_OK) 
    {
    Serial.println("Error initializing ESP-NOW");
    return;
    }
  
// Recieve Temp
        int tempMax = incomingTemp+1; //hotest allowable temperature
        int tempMin = incomingTemp-1; //coldest allowable temperature
        sensors.requestTemperatures(); 
        float temperatureC = sensors.getTempCByIndex(0);
        Serial.print(temperatureC);
        Serial.println("ºC");
        temperature = temperatureC;
        Serial.print ("Temperature is:");
        Serial.println (temperature);


  //Compare TEMPERATURE and execute ACTIONS
       //High Temperature
            if (temperature > tempMax)
            {
              digitalWrite(heater, LOW);  //Ensure the Temperature Increaser is OFF
              Serial.println("Water cooler ON");
                     
            }
        
       //Low Temperature
           if (temperature < tempMin)
           {
              digitalWrite(heater, HIGH);  // Turn the Temperture Increaser ON
              Serial.println("Water Heater is ON");
              Heater = 2500;
           }

//WATER LEVEL  
          water = analogRead(waterSensor);    // Read the analog value form sensor
          Serial.print ("Water Level is:");
          Serial.println (water);
          
 
   //Compare WATER and execute ACTIONS
        //High Water
            if (water > waterMax) 
            {
             digitalWrite(iFPower, LOW);  //Ensure the Inflow pump is OFF
              digitalWrite(oFPower, HIGH);  // Turn the Outflow pump ON
              Serial.println("OUTflow Water Pump is ON");
              Serial.println("INflow Water Pump is OFF"); 
              Outpump = 3500;
            }
   
        //Low Water
            if (water < waterMin) 
            {
             digitalWrite(oFPower, LOW);  //Ensure the Outflow pump is OFF
              digitalWrite(iFPower, HIGH);  // Turn the Inflow pump ON
              Serial.println("INflow Water Pump is ON"); 
              Serial.println("OUTflow Water Pump is OFF");
              Inpump = 4500;        
            }

 //pH Reading
     for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
      { 
        buf[i]=analogRead(SensorPin);
        delay(10);
      }
      for(int i=0;i<9;i++)        //sort the analog from small to large
      {
        for(int j=i+1;j<10;j++)
        {
          if(buf[i]>buf[j])
          {
            temp=buf[i];
            buf[i]=buf[j];
            buf[j]=temp;
          }
        }
      }
      avgValue=0;
      for(int i=2;i<8;i++)                      //take the average value of 6 center sample
        avgValue+=buf[i];
      float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
      phValue=3.5*phValue;
      Serial.println("phValue:");
      Serial.println(phValue);


//TDS Readings

   sensorValue = analogRead(sensorPin);
    Voltage = sensorValue*3.3/1024.0; //Convert analog reading to Voltage
    tdsValue=(133.42*Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5; //Convert voltage value to TDS value
    TDSg = tdsValue * TDSCon;
    //tdsValue = factor + TDS;
    //tdsValue = tds * TDSCon;
    Serial.println("TDS Value:");
    Serial.println(tdsValue);
     Serial.println(Voltage);
    Serial.println(sensorValue);

//package readings to send
              
  // Set values to send
  Serial.println("Sending the Readings");
  sendReadings.temp = temperature;
  sendReadings.wat = water;
  sendReadings.ph = phValue;
  sendReadings.turb = tdsValue;
  sendReadings.Heater = Heater;
  sendReadings.Inpump = Inpump;
  sendReadings.Outpump = Outpump;
  
 // Send message via ESP-NOW
  esp_err_t result = esp_now_send(hubAddress, (uint8_t *) &sendReadings, sizeof(sendReadings));

  if (result == ESP_OK) 
    {
      Serial.println("Reading Sent with success");
    }
  else 
    {
      Serial.println("Error sending the Readings");
    }

     delay(5000);
}



   
      
   
    
