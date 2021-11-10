
#include <SPI.h>
#include <RH_RF95.h>
#include <WiFiNINA_Generic.h>
#include <ArduinoJson.h>
#include <string>

#include "defines.h"
#include "arduino_secrets.h"

int SEESAW_NUM = 3;

// To eliminate FW warning when using not latest nina-fw version
// To use whenever WiFi101-FirmwareUpdater-Plugin is not sync'ed with nina-fw version
#define WIFI_FIRMWARE_LATEST_VERSION        "1.4.5"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP), length must be 8+

int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
char server[] = "httpbin.org";    // name address for server (using DNS)

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;

// Singleton instance of the radio driver
RH_RF95 rf95;

// single JSON instance we can write to over and over again
StaticJsonDocument<100> doc; // allocate on stack - we need about 100 bytes to have this working
StaticJsonDocument<100> doc_1; // allocate on stack - we need about 100 bytes to have this working

static int32_t twosComplement(int32_t val, uint8_t bits) {
  if (val & ((uint32_t)1 << (bits - 1))) {
    val -= (uint32_t)1 << bits;
  }
  return val;
}

void printWiFiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("Signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
}

void setup() 
{
  // initialize RF95
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed");  
  else
    Serial.println("initialization successful");

  /*

  // Initialize ESP32
  Serial.begin(115200);
  while (!Serial);

  Serial.print(F("\nStart WiFiWebClient on ")); Serial.println(BOARD_NAME);
  Serial.println(WIFININA_GENERIC_VERSION);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println(F("Communication with WiFi module failed!"));
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.print(F("Your current firmware NINA FW v"));
    Serial.println(fv);
    Serial.print(F("Please upgrade the firmware to NINA FW v"));
    Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED)
  {
    Serial.print(F("Attempting to connect to SSID: "));
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println(F("Connected to WiFi"));
  printWiFiStatus();
*/
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      if (len == 24) {
        // soil temperature calculation
        float soil_temp[SEESAW_NUM] = {0.0, 0.0, 0.0, 0.0};
        for (int i = 0; i < SEESAW_NUM; i++) {
          int32_t int_soil_temp = ((uint32_t)buf[4*i] << 24) | ((uint32_t)buf[1 + 4*i] << 16) |
                    ((uint32_t)buf[2 + 4*i] << 8) | (uint32_t)buf[3 + 4*i];
          soil_temp[i] = (1.0 / (1UL << 16)) * int_soil_temp;
        }

        // moisture calculation
        uint16_t moisture[SEESAW_NUM] = {0, 0, 0, 0};
        for (int i = 0; i < SEESAW_NUM; i++) {
          moisture[i] = ((uint16_t)buf[16 + (2*i)]) << 8 | (uint16_t)buf[17 + (2*i)];
        }

        doc["moisture_0"] = moisture[0];
        doc["moisture_1"] = moisture[1];
        doc["moisture_2"] = moisture[2];
        doc["moisture_3"] = moisture[3];
        doc["soil_temp_0"] = soil_temp[0];
        doc["soil_temp_1"] = soil_temp[1];
        doc["soil_temp_2"] = soil_temp[2];
        doc["soil_temp_3"] = soil_temp[3];

        String minif_json;
        serializeJson(doc, minif_json); // serialize object to String
        Serial.println(minif_json);
        
      } else if (len == 33) {

        // Pressure coefficient calculation
        int32_t c00 = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) |
               (((uint32_t)buf[2] >> 4) & 0x0F);
        c00 = twosComplement(c00, 20);
      
        int32_t c10 = (((uint32_t)buf[2] & 0x0F) << 16) | ((uint32_t)buf[3] << 8) |
               (uint32_t)buf[4];
        c10 = twosComplement(c10, 20);
      
        int16_t c01 = twosComplement(((uint16_t)buf[5] << 8) | (uint16_t)buf[6], 16);
        int16_t c11 = twosComplement(((uint16_t)buf[7] << 8) | (uint16_t)buf[8], 16);
        int16_t c20 = twosComplement(((uint16_t)buf[9] << 8) | (uint16_t)buf[10], 16);
        int16_t c21 = twosComplement(((uint16_t)buf[11] << 8) | (uint16_t)buf[12], 16);
        int16_t c30 = twosComplement(((uint16_t)buf[13] << 8) | (uint16_t)buf[14], 16);
  
        // pressure calculation
        int32_t raw_psr = twosComplement(((uint32_t)buf[15] << 16) | ((uint32_t)buf[16] << 8) | ((uint32_t)buf[17]), 24);
        int32_t raw_tmp = twosComplement(((uint32_t)buf[18] << 16) | ((uint32_t)buf[19] << 8) | ((uint32_t)buf[20]), 24);
  
        float scaled_rawtemp = (float)raw_tmp / 524288;
        float pressure = (float)raw_psr / 1572864;
  
        pressure =
            (int32_t)c00 +
            pressure * ((int32_t)c10 +
                         pressure * ((int32_t)c20 + pressure * (int32_t)c30)) +
            scaled_rawtemp *
                ((int32_t)c01 +
                 pressure * ((int32_t)c11 + pressure * (int32_t)c21));
  
        pressure = pressure / 100;
  
        // CO2, temp, humidity calculation
        union {
          uint32_t u32_value;
          float float32;
        } float_conversion;
  
        uint32_t co2_raw = (((uint32_t)buf[21]) << 24) + (((uint32_t)buf[22]) << 16) + (((uint32_t)buf[23]) << 8) + ((uint32_t)buf[24]);
        float_conversion.u32_value = co2_raw;
        float co2_conc = float_conversion.float32;
  
        uint32_t temp_raw = (((uint32_t)buf[25]) << 24) + (((uint32_t)buf[26]) << 16) + (((uint32_t)buf[27]) << 8) + ((uint32_t)buf[28]);
        float_conversion.u32_value = temp_raw;
        float temp = float_conversion.float32;
  
        uint32_t hum_raw = (((uint32_t)buf[29]) << 24) + (((uint32_t)buf[30]) << 16) + (((uint32_t)buf[31]) << 8) + ((uint32_t)buf[32]);
        float_conversion.u32_value = hum_raw;
        float hum_conc = float_conversion.float32;

        // JSON creation
        doc_1["pressure"] = pressure;
        doc_1["co2_concentration"] = co2_conc;
        doc_1["air_temp"] = temp;
        doc_1["humidity"] = hum_conc;

        String minif_json;
        serializeJson(doc_1, minif_json); // serialize object to String
        Serial.println(minif_json);
      }

/*
      String content_length_str = "Content-Length: ";
      content_length_str += minif_json.length();

      if (client.connect(server, 80))
      {
        Serial.println(F("Connected to server"));
        // Make a HTTP request:
        client.println("POST /post HTTP/1.1");
        client.println("Host: httpbin.org");
        client.println("Accept: application/json");
        client.println("Content-Type: application/json");
        client.println(content_length_str);
        client.println(minif_json);
        client.println("Connection: close");
        client.println();
      }
      Serial.println("Done.");

      // read response body
      while (client.available())
      {
        char c = client.read();
        Serial.write(c);
      }
    
      // if the server's disconnected, stop the client:
      if (!client.connected())
      {
        Serial.println(F("\nDisconnecting from server."));
        client.stop();
      }
*/
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}
