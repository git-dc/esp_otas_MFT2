

/*********
 Daniel Chirita
  Using Rui Santos's OTA code (Complete project details at http://randomnerdtutorials.com).
*********/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define WIFI_SSID "2.4g"
#define WIFI_PSK "dauyndauyn"
#define OTA_PSK "dauyndauyn"
#define HOSTNAME "ftp_meter_0"
#define MQTT_SERVER "house"
#define MQTT_TOPIC "FTP"
#define MQTT_IN_TOPIC "inTopic"

// Update these with values suitable for your network.

// cnk vars begin:
unsigned long currentTime = millis();
unsigned long DebugloopTime = currentTime;
bool debug = false;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PSK;
const char* ota_password = OTA_PSK;
  // mqtt setup begin:
  const char* mqtt_server = MQTT_SERVER;
  const char* mqtt_topic = MQTT_TOPIC;
  char msg[50];
  WiFiClient espClient;
  PubSubClient client(espClient);
  //mqtt setup end;
// cnk vars end;

// blink vars begin:
const int ESP_BUILTIN_LED = 2; // 2- blue, 0 - red
unsigned long thisLoop=millis();
unsigned long interval = 750;
bool ledState = HIGH;
bool blinking = false;
// blink vars end;

// flow vars begin:
volatile int flow_frequencyC; // Measures flow sensor pulses
volatile int flow_frequencyH; // Measures flow sensor pulses
double l_min; // Calculated litres/hour
unsigned char flowsensorC = 13; // cold pipe (13 for flow, adc0 for temp)
unsigned char flowsensorH = 12; // hot pipe (12 for flow, adc1 for temp)
unsigned long FloopTime = currentTime;;
// flow vars end;


// temp vars begin:
double tempC; // cold pipe (13 for flow, adc0 for temp)
double tempH; // hot pipe (12 for flow, adc1 for temp)
double Rs = 470000.0;
double Vcc = 3.3;
unsigned long TloopTime = currentTime;;
// temp vars end;

// pressure vars begin:
double presC; // cold pipe (13 for flow, adc0 for temp)
double presH; // hot pipe (12 for flow, adc1 for temp)
unsigned long PloopTime = currentTime;;
// pressure vars end;


// secup vars begin:
const int updateButton=0; // gpio0 button (red led)
bool updateButtonState; // active low
unsigned long secupTime;
bool secup = false;
// secup vars end;


// adc vars begin:
Adafruit_ADS1115 adc(0x48);
int16_t adc0, adc1, adc2, adc3;
// adc vars end;


void flowC (){flow_frequencyC++;} // Interrupt function
void flowH (){flow_frequencyH++;} // Interrupt function

void setup_wifi() 
{
  delay(10);
  // wifi network cnk begin:
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.hostname(HOSTNAME);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  
  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // wifi network cnk end;
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
    //digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    //String clientId = "esp_client_";
    String clientId = HOSTNAME;
    //clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      sprintf(msg, "%s reconnected.", clientId.c_str());
      client.publish(mqtt_topic, msg);
      // ... and resubscribe
      client.subscribe(MQTT_IN_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}


void setup() {
////////////////////////////////////////////////////////
//////////////// WIFI - OTA SETUP BEGIN ////////////////
////////////////////////////////////////////////////////
  delay(100);
  Serial.begin(115200);
  Serial.println("Booting");
  setup_wifi();
  
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  //ArduinoOTA.setHostname("esp_otas_MFT2");
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword((const char *) ota_password);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

///////////////////////////////////////////////////////
////////////// WIFI - OTA SETUP COMPLETE //////////////
///////////////////////////////////////////////////////


  client.setServer(mqtt_server, 1883);
  
  client.setCallback(callback);
  pinMode(flowsensorC, INPUT);
  digitalWrite(flowsensorC, HIGH); // Optional Internal Pull-Up
  attachInterrupt(flowsensorC, flowC, RISING); // Setup Interrupt
  pinMode(flowsensorH, INPUT);
  digitalWrite(flowsensorH, HIGH); // Optional Internal Pull-Up
  attachInterrupt(flowsensorH, flowH, RISING); // Setup Interrupt
  sei(); // Enable interrupts
  
  
  if (secup){pinMode(updateButton, INPUT);}

  adc.begin();
  // Set the gain to 4x, for an input range of +/- 1.024V
  // 1-bit = 0.5V on the ADS1015 with this gain setting
  adc.setGain(GAIN_FOUR);

  if (not blinking){digitalWrite(ESP_BUILTIN_LED, HIGH);}else{pinMode(ESP_BUILTIN_LED, OUTPUT);}
  
}

void loop() {
   
  currentTime = millis();
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
    
  // blink blurb begin:
  if (blinking == true and (currentTime - thisLoop >= interval)){
    thisLoop=currentTime;
    if (ledState == LOW) { ledState = HIGH; } else ledState = LOW;
    digitalWrite(ESP_BUILTIN_LED, ledState);
  }  
  // blink blurb end;


  // secup blurb begin:
  if(secup){
    if (currentTime >= (secupTime + 20000) or updateButtonState) // 20 seconds to auth and upload
    { 
      updateButtonState = digitalRead(updateButton); 
      if (not updateButtonState){
        sprintf(msg, "Update button activated.");
        Serial.println(msg);
        client.publish(mqtt_topic, msg); 
      }
    secupTime=currentTime;
    }
    if (not updateButtonState){ArduinoOTA.handle();} // secup: press down gpio0 button while uploading;
  }
  else ArduinoOTA.handle(); // no secup
  // secup blurb end;

  
  
  if(currentTime >= (TloopTime + 1000))
  { 
    TloopTime=currentTime;  
    
    // tempC loop blurb begin:
    adc0 = adc.readADC_SingleEnded(0);
    tempC = 263.0 - 22.0*log( Rs / ( ( ( Vcc * 32767.0 ) / adc0 ) - 1.0 ) ); // based on 50kOhm B=3995 thermistor exponential approximation curve
    sprintf(msg, "%s tempC C: %d.%d", HOSTNAME, int(tempC), int(tempC * 100) %100);
    Serial.println(msg); client.publish(mqtt_topic, msg); 
    // tempC loop blurb end;

    // tempH loop blurb begin:
    adc1 = adc.readADC_SingleEnded(1);
    tempH = 263.0 - 22.0*log( Rs / ( ( ( Vcc * 32767.0 ) / adc1 ) - 1.0 ) ); // based on 50kOhm B=3995 thermistor exponential approximation curve
    sprintf(msg, "%s tempH C: %d.%d", HOSTNAME, int(tempH), int(tempH * 100) %100);
    Serial.println(msg); client.publish(mqtt_topic, msg); 
    // tempH loop blurb end;
  }


  
  if(currentTime >= (FloopTime + 1000))
  { 
    FloopTime=currentTime;  
    
    // flowC loop blurb begin:
    l_min = (flow_frequencyC / 11.0); // (Pulse frequency (Hz) / 11) = flowrate in L/min for YF-B7 flow sensor.
    flow_frequencyC = 0; // Reset Counter
    sprintf(msg, "%s flowC L/min: %d.%d", HOSTNAME, int(l_min), int(l_min*100) %100);
    Serial.println(msg); client.publish(mqtt_topic, msg);  
    // flowC loop blurb end;

    // flowH loop blurb begin:
    l_min = (flow_frequencyH / 11.0); // (Pulse frequency (Hz) / 11) = flowrate in L/min for YF-B7 flow sensor.
    flow_frequencyH = 0; // Reset Counter
    sprintf(msg, "%s flowH L/min: %d.%d", HOSTNAME, int(l_min), int(l_min*100) %100);
    Serial.println(msg); client.publish(mqtt_topic, msg);  
    // flowH loop blurb end;
  }


if(currentTime >= (PloopTime + 1000))
  { 
    PloopTime=currentTime;  
    
    // presC loop blurb begin:
    adc3 = adc.readADC_SingleEnded(3);
    //presC = 263.0 - 22.0*log( Rs / ( ( ( Vcc * 32767.0 ) / adc3 ) - 1.0 ) ); // based on 50kOhm B=3995 thermistor exponential approximation curve
    presC = 200*adc3/65535.0;
    sprintf(msg, "%s presC psi: %d.%d", HOSTNAME, int(presC), int(presC * 100) %100);
    Serial.println(msg); client.publish(mqtt_topic, msg); 
    // presC loop blurb end;

    // presH loop blurb begin:
    adc2 = adc.readADC_SingleEnded(2);
    //presH = 263.0 - 22.0*log( Rs / ( ( ( Vcc * 32767.0 ) / adc2 ) - 1.0 ) ); // based on 50kOhm B=3995 thermistor exponential approximation curve
    presH = 200*adc2/65535.0;
    sprintf(msg, "%s presH psi: %d.%d", HOSTNAME, int(presH), int(presH * 100) %100);
    Serial.println(msg); client.publish(mqtt_topic, msg); 
    // presH loop blurb end;
  }

  
  if(currentTime >= (DebugloopTime + 1000000))
  { 
    DebugloopTime=currentTime;  
   
    // debug loop blurb begin:
    sprintf(msg, "debug info: %d.%d", int(Vcc), int(Vcc * 100) %100);
    //Serial.println(msg); client.publish(mqtt_topic, msg); 
    // debug loop blurb end;
    
  }

}


