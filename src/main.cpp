#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>

/* Network definitions */
#define WIFISSID ""
#define PASSWORD ""
#define TOKEN  ""
#define MQTT_CLIENT_NAME ""

/* Constants definitions */
#define VARIABLE_LABEL "ad8232"
#define DEVICE_LABEL "esp32"

/* GPIO definitions */
#define SENSOR_ADC_PIN        A0
#define SENSOR_SHD_PIN        35
#define SENSOR_LO_MINUS_PIN   32
#define SENSOR_LO_PLUS_PIN    34

/****************************** WIFI Client **********************************************/
WiFiClient wifi_client;
WiFiUDP    wifi_NtpUDP;

/***************************** MQTT Client  *********************************************/

/*** ubidots ****/
// #define MQTT_BROKER       "industrial.api.ubidots.com";
// #define MQTT_CLIENT_NAME  ""
// #define TOKEN   ""

/*** mosquitto ****/
#define MQTT_BROKER      ""

/*** hivemq ****/
// #define MQTT_BROKER ""

#define MQTT_CLIENT_NAME  
#define MQTT_PASSWORD     
#define MQTT_PORT         1883

PubSubClient client(wifi_client);
char mqttBroker[] = MQTT_BROKER;
char topic[150];

NTPClient timeClient(wifi_NtpUDP, "pool.ntp.org");

char str_millis[20];
double epochseconds = 0;
double epochmilliseconds = 0;
double current_millis = 0;
double current_millis_at_sensordata = 0;
double timestampp = 0;

/******************************** Filter section ****************************************/
#include <filters.h> // must be loaded manually
#include <KickFiltersRT.h>

const float flp_cutoff_freq  = 30.0;                    //Cutoff frequency in Hz
const float fhp_cutoff_freq  = 0.05;                    //Cutoff frequency in Hz
const float sampling_time    = 0.02;                    //Sampling time in seconds // se poate de modificat cu cit e mai mare cu atat mai mult zgomot
const float sampling_time_ms = sampling_time * 1000;    //Sampling time in milliseconds

// Low-pass filter
Filter flp(flp_cutoff_freq, sampling_time, IIR::ORDER::OD3);
// High pass filter
Filter fhp(fhp_cutoff_freq, sampling_time, IIR::ORDER::OD2, IIR::TYPE::HIGHPASS);

// Notch filter 50 Hz
const float notch_freq = 50.0f;
KickFiltersRT<float> fnch;

float samples_buff[20]; // 100 
char payload[10000];
uint16_t buff_index = 0;

/******************************** Functions definition **********************************/

/* System setup */
void setup() 
{
  Serial.begin(250000);

  // Analog input
  pinMode(SENSOR_ADC_PIN, INPUT);
  // Shudown output - inversed
  pinMode(SENSOR_SHD_PIN, OUTPUT);
  // Lead Off Negative - input
  pinMode(SENSOR_LO_MINUS_PIN, INPUT);
  // Lead Off Positive - input
  pinMode(SENSOR_LO_PLUS_PIN, INPUT);

  // Shudown output - inversed
  digitalWrite(SENSOR_SHD_PIN, HIGH);

  // Init notch filter
  fnch.initnotch(0, 0, notch_freq, 1.0f / sampling_time);

  // Start Wi-Fi connection   
  WiFi.begin(WIFISSID, PASSWORD);
  Serial.println();
  Serial.print("Waiting for WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Set MQTT Broker
  timeClient.begin();
  client.setServer(mqttBroker, MQTT_PORT);
  timeClient.update();

  // Current time
  epochseconds = timeClient.getEpochTime();
  epochmilliseconds = epochseconds * 1000;
  Serial.print("epochmilliseconds=");
  Serial.println(epochmilliseconds);
  current_millis = millis();
}

void reconnect()
{
  while (!client.connected()) {

    Serial.println("Attempting MQTT connection...");

    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

/* Main loop */
void loop() 
{
  static unsigned long previousMillis = 0UL;

  if (!client.connected()) {
    reconnect();
  }

  unsigned long currentMillis = millis();

  if((currentMillis - previousMillis) >= (unsigned long)sampling_time_ms)
  {
	  previousMillis = currentMillis;
  
    float adc_sample = 0.0;

    if ((LOW == digitalRead(SENSOR_LO_MINUS_PIN)) && (LOW == digitalRead(SENSOR_LO_PLUS_PIN))) {
        adc_sample = (float)analogReadMilliVolts(SENSOR_ADC_PIN) / 100.0f;
    }
    
    // Remove DC signal component
    float filtered_val_fnch = fnch.notch(adc_sample);
    float filtered_val_fhp  = fhp.filterIn(adc_sample);
    float filtered_val_flp  = flp.filterIn(filtered_val_fhp);
    
    if (buff_index < (sizeof(samples_buff) / sizeof(samples_buff[0]))) {
      samples_buff[buff_index] = filtered_val_flp;
      buff_index++;
    }

    if (buff_index >= (sizeof(samples_buff) / sizeof(samples_buff[0]))) {
      buff_index = 0;
      
      sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
      sprintf(payload, "%s", "");
      sprintf(payload, "{\"%s\": [", VARIABLE_LABEL);

      char str_sensor[6];

      for (int i = 0; i < (sizeof(samples_buff) / sizeof(samples_buff[0])); i++) {
        dtostrf(samples_buff[i], 4, 2, str_sensor);
        sprintf(payload, "%s{\"value\":", payload);
        sprintf(payload, "%s %s,", payload, str_sensor);
        // sprintf(payload, "%s},", payload);

        current_millis_at_sensordata = millis();
        timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
        dtostrf(timestampp, 10, 0, str_millis);
        sprintf(payload, "%s \"timestamp\": %s},", payload, str_millis); // Adds the value

        Serial.println(samples_buff[i]);
      }

      current_millis_at_sensordata = millis();
      timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
      dtostrf(timestampp, 10, 0, str_millis);
      sprintf(payload, "%s{\"value\":%s, \"timestamp\": %s}]}", payload, str_sensor, str_millis);
      //Serial.println(payload);
      
      client.publish(topic, payload);
    }
  }

  //client.loop();
}


