#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char *ssid = "ESP32_S3_AP";
const char *password = NULL; // No password for this AP

const char *serverUrl = "http://192.168.4.1/sensors";

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    delay(1000);

    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;

        Serial.print("Making request to: ");
        Serial.println(serverUrl);

        http.begin(serverUrl);
        int httpCode = http.GET();

        if (httpCode > 0)
        {
            Serial.printf("[HTTP] GET... code: %d\n", httpCode);

            if (httpCode == HTTP_CODE_OK)
            {
                String payload = http.getString();
                Serial.println("Received payload:");
                
                // Pretty print JSON
                DynamicJsonDocument doc(1024);
                DeserializationError error = deserializeJson(doc, payload);

                if (error) {
                    Serial.print(F("deserializeJson() failed: "));
                    Serial.println(error.c_str());
                    Serial.println("Printing raw payload instead:");
                    Serial.println(payload);
                } else {
                    serializeJsonPretty(doc, Serial);
                    Serial.println();
                }
            }
            else
            {
                Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
            }
        }
        else
        {
            Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        http.end();
    }
    else
    {
        Serial.println("WiFi Disconnected. Trying to reconnect...");
        WiFi.begin(ssid, password);
        // Wait a moment before retrying
        delay(1000);
    }

    Serial.println("\nWaiting 5 seconds before next request...");
    delay(5000);
}
