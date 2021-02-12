#include <global.h>
#include <base64.h>
#include <WebSocketClient.h>
#include <WebSocketServer.h>
#define _WIFININA_LOGLEVEL_       1
#include <WiFiNINA_Generic.h>

#include "arduino_secrets_private.h"
#include "network_parameters.h"

// global var
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
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
    IP[0] = MYHOME_IP_OCT1;
    IP[1] = MYHOME_IP_OCT2;
    IP[2] = MYHOME_IP_OCT3;
    IP[3] = MYHOME_IP_OCT4;
  
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
