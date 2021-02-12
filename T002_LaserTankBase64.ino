#include <global.h>
#include <base64.h>
#include <WebSocketClient.h>
#include <WebSocketServer.h>
#define _WIFININA_LOGLEVEL_       1
#include <WiFiNINA_Generic.h>

#include "arduino_secrets_private.h"

// global var
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
const byte IPLastByte  = 99;
const short webPort     = 80;
const short socketPort  = 8080;

// Background
boolean Backgroundinit = true;
boolean BackgroundHearBeat = false;

// WIFI objects
WiFiServer      webServer(webPort);
WiFiServer      socketServer(socketPort);
WebSocketServer webSocketServer;
WiFiClient      socketClient;

// Laser Tank webpage, gzipped and base64 encoding to save space
char webpage_base64[] = "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";

void printWifiStatus() 
{
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("Signal strength (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("Gateway: "); Serial.println(WiFi.gatewayIP());
    Serial.print("Netmask: "); Serial.println(WiFi.subnetMask());
    Serial.print("Webpage is at http://"); Serial.print(WiFi.localIP()); Serial.println("/");
    Serial.print("Websocket is at http://"); Serial.print(WiFi.localIP()); Serial.println(":" + (String)socketPort + "/");
}

void WiFiConnect() 
{
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Connecting to " + (String)ssid + " ...");
        WiFi.begin(ssid, pass);
        delay(5000);
    }
  
    IPAddress IP = WiFi.localIP();
    IP[3] = IPLastByte;
  
    WiFi.config(IP, WiFi.gatewayIP(), WiFi.gatewayIP(), WiFi.subnetMask());
    Serial.println("Connected to " + (String)ssid);

    webServer.begin();
    socketServer.begin();
    printWifiStatus();
    WiFi.lowPowerMode();
}

void setup()
{
    // we use this led to show the background is running
    // flash on and off every time we go through the background loop
    pinMode(LED_BUILTIN, OUTPUT);

    // Serial port initialization
    Serial.begin(57600);

    Serial.println("\nStart MultiServers");
    Serial.println("Version " + String(WIFININA_GENERIC_VERSION));

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    {
        Serial.print("Your current firmware NINA FW v");
        Serial.println(fv);
        Serial.print("Please upgrade the firmware to NINA FW v");
        Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
    }

    // setup complete
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
    if(WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Lost WiFi connection");
        WiFi.end();
        WiFiConnect();
    }

    WiFiClient webClient = webServer.available();
  
    if(webClient.connected())
    {
        Serial.print("New client: "); Serial.print(webClient.remoteIP()); Serial.print(":"); Serial.println(webClient.remotePort());

        String header = "";
  
        while(webClient.available())
        {
            char ch = webClient.read();
  
            if (ch != '\r') 
            {
            header += ch;
            }
        
            if (header.substring(header.length() - 2) == "\n\n") 
            {
                Serial.print(header.substring(0, header.length() - 1));

                if (header.indexOf("GET / HTTP") > -1) 
                {
                    const int webpage_base64_length = sizeof(webpage_base64);
                    const int webpage_gz_length = base64_dec_len(webpage_base64, webpage_base64_length);
                    char webpage_gz[webpage_gz_length];
                    base64_decode(webpage_gz, webpage_base64, webpage_base64_length);
                    int packetsize = 1024;
                    int done = 0;
                    
                    webClient.println("HTTP/1.1 200 OK\nContent-Type: text/html\nContent-Encoding: gzip\n");
                
                    while (webpage_gz_length > done) 
                    {
                        if (webpage_gz_length - done < packetsize) {
                            packetsize = webpage_gz_length - done;
                        }
              
                        webClient.write(webpage_gz + done, packetsize * sizeof(char));
                        done = done + packetsize;
                    }

                    Serial.println("--Interface webpage sent");
                 } 
                else 
                {
                    webClient.println("HTTP/1.1 404 Not Found\nContent-Type: text/plain; charset=utf-8\n\n404 Not Found\n");
                    Serial.println("--Page not found");
                }
          
                webClient.stop();
                Serial.println("--Client disconnected");
            }
        }
    }

    if(!socketClient.connected()) 
    {
        socketClient = socketServer.available();
    
        if (socketClient.connected() && webSocketServer.handshake(socketClient)) 
        {
            Serial.print("Websocket connected to: ");
            Serial.print(socketClient.remoteIP());
            Serial.print(":");
            Serial.println(socketClient.remotePort());
        } 
        else 
        {
            socketClient.stop();
            delay(100);
        }
    }

    if(socketClient.connected()) 
    {
        // Background Init - setup the background tasks, runs only once
        if(Backgroundinit == true) {
            Backgroundinit = false;
            String data = webSocketServer.getData();
            Serial.println("Websocket Flushed");
            Serial.println("Background Init Complete");
        }
    
        // Background Process 1
        // see if we have a command/request from the user 
        String data = webSocketServer.getData();
        if (data.length() > 0) 
        {
            String cmd = data.substring(0, data.indexOf(":"));
            String setting = data.substring(data.indexOf(":") + 1);
    
            // process command
            switch (cmd.toInt()) {
            case 1:
                // Forward
                break;
            case 2:
                // Reverse
                break;
            case 3:
                // Max Forward
                break;
            case 4:
                // Stop
                break;
            case 5:
                // Max Reverse
                break;
            case 6:
                // Left
                break;
            case 7:
                // Right
                break;
            case 8:
                // Max Left
                break;
            case 9:
                // Straight
                break;
            case 10:
                // Max Right
                break;
            case 11:
                // Target LED
                webSocketServer.sendData("R:Laser Targeting");
                break;
            case 12:
                // Laser Left
                break;
            case 13:
                // Laser Right
                break;
            case 14:
                // Laser Up
                break;
            case 15:
                // Laser Down
                break;
            case 16:
                // Fire
                webSocketServer.sendData("R:Laser Fired");
                break;
            case 17:
                // Send request/command to secondary processor
                break;
            case 18:
                // This is the -Battery+ button
                break;
            case 19:
                // Process a Primary command
                break;
            case 20:
                // Speed UP
                break;
            case 21:
                // Slow Down
               break;
            default:
                webSocketServer.sendData("E:" + cmd + " - " + setting);
                break;
            }
        }
    }

    // Background Process 2
    // show the background heart
    BackgroundHearBeat = !BackgroundHearBeat;
    if(BackgroundHearBeat) {
        digitalWrite(LED_BUILTIN, LOW);  
    } else {
        digitalWrite(LED_BUILTIN, HIGH);  
    }
}
