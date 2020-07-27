#include <ArduinoJson.h>
#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

#define buzzer 18// port for the warning sound (BUZZER)

//#include <string>
int tank_id = 1;

// ACCESS POINT credentials
// Wi-Fi channel for the access point (must match the sender channel)
#define CHAN_AP 1

// REPLACE WITH THE MAC Address of your receiver 
  uint8_t tempAddress[] = {0x24, 0x6f, 0x28, 0xB0, 0x92, 0x80}; //Temperature Board

//Wifi and server setup
  const char* ssid = "Corals";
  const char* password = "coralconnect";
  char* sensorData = "http://192.168.43.234:8081/sensordata";
  char* activityLog = "http://192.168.43.234:8081/createActivityLog";
  char* GetRequest = "http://192.168.43.234:8081/configurationsGet/?tankid=1";  


// Define variables to store user readings to be sent
  float temperature;
  float water;
  float pH;
  float turbidity;

// Define variables to store incoming readings
  float incomingTemp;
  float incomingWater;
  float incomingPH;
  float incomingTurb;
  float incomingHeater;
  float incomingInpump;
  float incomingOutpump;


// Define variables to store incoming Activities Log in TEXT

    String Heaterlog;
    String Inpumplog;
    String Outpumplog;
    
// apiKeyValue value must match the PHP file /post-data.php 
  String data_string = "This data is being sent to the DB:";

//dummy vatlues
  float incomingSalinity = 34;
  float Blank = 0;
   
// Variable to store if sending data was successful
  String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float temp;
    float wat;
    float ph;
    float turb;
    float Heater;
    float Inpump;
    float Outpump;
} struct_message;

// Create a struct_message to hold user measures that will be sent to the boards
struct_message userMeasures;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

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

//warning checks
  int tempWarn = 0;
  int watWarn = 0;
  int phWarn = 0;
  int turbWarn = 0;


// Callback when data is received
  void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
    {
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    Serial.print("Bytes received: ");
    Serial.println(len);
  
      //Values coming from the child board
      incomingTemp = incomingReadings.temp;
      incomingWater = incomingReadings.wat;
      incomingPH = incomingReadings.ph;
      incomingTurb = incomingReadings.turb;
      incomingHeater = incomingReadings.Heater;
      incomingInpump = incomingReadings.Inpump;
      incomingOutpump = incomingReadings.Outpump;
      
      //converting activities to text 
        //Heater
            if (incomingHeater>1100)
              {
                Heaterlog = "Low Temp, Water Heater turned ON";
              }
              else
              {
                Heaterlog = "Water Heater is OFF";
              }
      
        //Inflow Pump
            if (incomingInpump>1100)
              {
                Inpumplog = "Low Water Level, Inflow Water Pump turned ON";
              }
              else
              {
                Inpumplog = "Inflow Water Pump is OFF";
              }
      
        //Outflow Pump
            if (incomingOutpump>1100)
              {
                Outpumplog = "High Water Level,Outflow Water Pump turned ON";
              }
              else
              {
                Outpumplog = "Outflow Water Pump is OFF";
              }

        
      Serial.println ("Recieved Temperature:");
      Serial.println (incomingTemp);
      Serial.println ("Recieved Water Level:");
      Serial.println (incomingWater);
      Serial.println ("Recieved pH:");
      Serial.println (incomingPH);
      Serial.println ("Recieved TDS:");
      Serial.println (incomingTurb);
      Serial.println ("The Heater Code is:");
      Serial.println (incomingHeater);
      Serial.println ("The INpump Code is:");
      Serial.println (incomingInpump);
      Serial.println ("The OUTpump Code is:");
      Serial.println (incomingOutpump);
    }
   
void setup() 
{
  pinMode (buzzer, OUTPUT);
  
  // Init Serial Monitor
  Serial.begin(115200);

  Serial.println("Ready");    //Test the serial monitor
  
  WiFi.begin(ssid, password);

  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  Serial.print("Connected to network named: ");
  Serial.print(ssid);
     
  // Set device as a Wi-Fi Station  
  WiFi.mode(WIFI_AP_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, tempAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return;
    }
    else 
    {
      Serial.println("Peer Added Successfully!");
    }
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() 
{

    //Check WiFi connection status
 if(WiFi.status()== WL_CONNECTED)
  {
    HTTPClient http;   //declare http client

    //Domain name with URL path or IP address with path
    http.begin(sensorData); //start the route for the sensor data
  
    //packaging readings to be sent to the DB
    String httpRequestData =   "data_string=" + data_string + "&temperature=" + (incomingTemp) + "&TDS=" + (incomingTurb)+ "&salinity=" + (incomingSalinity)+ "&ph=" + (incomingPH) + "&waterlevel=" + (incomingWater) + "&tank_id=" + (tank_id);

    // Specification of content-type header
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    Serial.print("httpRequestData: ");
    Serial.println(httpRequestData);
  
     // Send HTTP POST request
      int httpResponseCode = http.POST(httpRequestData);

          if (httpResponseCode>0) 
              {
              Serial.print("HTTP Response code: ");
              Serial.println(httpResponseCode);
              Serial.println(http.getString());
              }
              else 
              {
                Serial.print("Error code: ");
                Serial.println(http.errorToString(httpResponseCode));
              }
    // Free resources
    http.end();
  

 //Sending Activities log
    //Domain name with URL path or IP address with path
    HTTPClient httpActivity;   //declare http client
    http.begin(activityLog);
      
    //packaging readings to be sent to the DB
    String httpRequestHeater =   "activity=" + (Heaterlog) + "&tank_id=" + (tank_id);
    String httpRequestInpump =   "activity=" + (Inpumplog) + "&tank_id=" + (tank_id);
    String httpRequestOutpump =  "activity=" + (Outpumplog) + "&tank_id=" + (tank_id);
    
     // Specification of content-type header
     http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
      Serial.println("httpRequests for Activities ");
      Serial.println(httpRequestHeater);
      Serial.println(httpRequestInpump);
      Serial.println(httpRequestOutpump);
      
       // Send HTTP POST request
        int httpResponseCodeOutpump = http.POST(httpRequestOutpump);
        delay(750);

         // Specification of content-type header
     http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        int httpResponseCodeHeater = http.POST(httpRequestHeater);
        delay(750);

         // Specification of content-type header
     http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        int httpResponseCodeInpump = http.POST(httpRequestInpump);
        
    
      if (httpResponseCodeHeater>0) 
        {
          Serial.print("HTTP HEATER Response code: ");
          Serial.println(httpResponseCodeHeater);
          Serial.println(http.getString());
        }
        else 
          {
            Serial.print("HEATER Error code: ");
            Serial.println(http.errorToString(httpResponseCodeHeater));
          }

      if (httpResponseCodeInpump>0) 
        {
          Serial.print("HTTP INPUMP Response code: ");
          Serial.println(httpResponseCodeInpump);
          Serial.println(http.getString());
        }
        else 
          {
            Serial.print("INPUMP Error code: ");
            Serial.println(http.errorToString(httpResponseCodeInpump));
          }

      if (httpResponseCodeOutpump>0) 
        {
          Serial.print("HTTP OUTPUMP Response code: ");
          Serial.println(httpResponseCodeOutpump);
          Serial.println(http.getString());
        }
        else 
          {
            Serial.print("INPUMP Error code: ");
            Serial.println(http.errorToString(httpResponseCodeOutpump));
          }
        
        // Free resources
        httpActivity.end();
  
  //USER CONFIG REQUEST HANDLER //
      HTTPClient httpGet;   //declare http client

    //Domain name with URL path or IP address with path
      httpGet.begin(GetRequest);

      httpGet.addHeader("Content-Type", "application/json","charset=utf-8");   

      String tankID = String("{\"tankid\":") + (tank_id) +  String("}"); 

      int GetResponseCode =  httpGet.POST(tankID);  // Make the request
      
      String ConfigValues = httpGet.getString();
      
      //JSON Parser//
      StaticJsonDocument<250> doc;
      DeserializationError err = deserializeJson(doc, ConfigValues);
            
      if (err)
        {
          Serial.print("ERROR: ");
          Serial.print(err.c_str());
          return;
       }
      //END JSON Parser//

       temperature = doc["temperature"];
        water = doc["waterlevel"];
        pH = doc["ph"];
        turbidity= doc["TDS"];
    
       Serial.print("User Temperature is: ");
       Serial.println(temperature); 
       Serial.print("User Water Level is: ");
       Serial.println(water);
       Serial.print("User pH is: ");
       Serial.println(pH); 
       Serial.print("User TDS is: ");
       Serial.println(turbidity); 

       
      if (GetResponseCode > 0)
      {        
        Serial.print("HTTP GET Response code: ");
        Serial.println(GetResponseCode);
        Serial.println(httpGet.getString());
        Serial.println(ConfigValues);
      }
     else 
      {
       Serial.print("Error code: ");
       Serial.println(httpGet.errorToString(GetResponseCode));
      }

        // Free resources
      httpGet.end();
  
  }
  
 if(WiFi.status()!= WL_CONNECTED) 
   {
     Serial.println("WiFi Disconnected");
   }
 
 //Trigger warnings
      if (incomingTemp != temperature)
        {
          tempWarn++;
          Serial.println(tempWarn);
        }
        if (tempWarn == 4)
          {
            warning();
            tempWarn=0;
          }
    
      if (incomingWater != water)
        {
          watWarn++;
          Serial.println(watWarn);
        }
          if (watWarn == 4)
          {
            warning();
            watWarn=0;
          }
    
      if (incomingPH != pH)
        {
          phWarn++;
          Serial.println(phWarn);
        }
          if (phWarn == 4)
          {
            warning();
            phWarn=0;
          }
    
      if (incomingTurb != turbidity)
        {
          turbWarn++;
          Serial.println(turbWarn);
        }
          if (turbWarn == 4)
          {
            warning();
            turbWarn=0;
          }
        
 
  // Set values to send
      userMeasures.temp = temperature;
      userMeasures.wat = water;
      userMeasures.ph = pH;
      userMeasures.turb = turbidity;
      userMeasures.Heater = Blank; 
      userMeasures.Inpump = Blank;
      userMeasures.Outpump = Blank;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(tempAddress, (uint8_t *) &userMeasures, sizeof(userMeasures));
   
  if (result == ESP_OK) 
    {
      Serial.println("User configuration Sent with success");
    }
  else
    {
      Serial.println("Error sending the User configuration");
    }
   delay(15000);
}
 
//Warning
    int warning()
      {
      for (int w = 0; w < 4; w++) // sound horn 4 times
        {
        Serial.println ("WARNING!!!");
        digitalWrite (buzzer, HIGH);
        delay(1500);
        digitalWrite (buzzer, LOW);
        delay(750);  
        }
      }
    
  
  
