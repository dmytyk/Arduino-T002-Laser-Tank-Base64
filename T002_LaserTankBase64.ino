#include <global.h>
#include <base64.h>
#include <WebSocketClient.h>
#include <WebSocketServer.h>
#define _WIFININA_LOGLEVEL_       1
#include <WiFiNINA_Generic.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "arduino_secrets.h"

// Console Attached
#ifndef TerminalAttached
    // true = terminal attached (send serial messages), false = no terminal attached no messages 
    #define TerminalAttached  false
#endif

// global var
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
const byte IPLastByte  = 99;
const short webPort     = 80;
const short socketPort  = 8080;

// pin assignments
#ifndef TargetLED
    // A1/D16 - Blue LED
    #define TargetLED  16
#endif
#ifndef FireLED
    // A2/D17 - Red LED
    #define FireLED  17
#endif

#ifndef VoltageMonitorPin
    // A6 / D21
    #define VoltageMonitorPin  A6
#endif

#ifndef LowVoltageCutoff
    // the value we want keep the system from starting and/or when to shutdown
    #define LowVoltageCutoff  10.5
#endif

#ifndef LowVoltagePin
    // D6 - Red LED
    #define LowVoltagePin  7
#endif

#ifndef LASERDIGITALPWM
    // D5 - PWM Out to TTL of Laser
    #define LASERDIGITALPWM  5
#endif
#ifndef MRen1
    // A3/D18 - Motor 1 PWM
    #define MRen1  18
#endif
#ifndef MRin1
    // 1 - Motor 1 Direction 1
    #define MRin1  1
#endif
#ifndef MRin2
    // 2 - Motor 1 Direction 2
    #define MRin2  2
#endif
#ifndef MLen1
    // A4/D19 - Motor 2 PWM
    #define MLen1  19
#endif
#ifndef MLin1
    // 3 - Motor 1 Direction 1
    #define MLin1  3
#endif
#ifndef MLin2
    // 4 - Motor 2 Direction 2
    #define MLin2  4
#endif
#ifndef TankMinSpeed
    // Minimum Speed
    #define TankMinSpeed  100
#endif
#ifndef TankMaxSpeed
    // Maximun Speed
    #define TankMaxSpeed  250
#endif
#ifndef TankSwitchTime
    // Delay time to Switch Direction, allow motors to slow down first so they are not hammered by the direction switch
    #define TankSwitchTime  500
#endif
#ifndef SERVO_MIN_PULSE_WIDTH
    // PCA9685 minimum pulse with to servos
    #define SERVO_MIN_PULSE_WIDTH   600
#endif
#ifndef SERVO_MAX_PULSE_WIDTH
    // PCA9685 maximum pulse with to servos
    #define SERVO_MAX_PULSE_WIDTH   2400
#endif
#ifndef SERVO_FREQUENCY
    // PCA9685 frequency of pulses to servos
    #define SERVO_FREQUENCY 50
#endif
#ifndef LASECENTERLR
    // the center of the left/right position
    #define LASECENTERLR 1630
#endif
#ifndef LASEMAXUD
    // the maximum up position
    #define LASEMAXUD 1500
#endif

// motor speed control values, these are set as commands come in from client
// Motor control range is 0 : 250 (0 - 5 or 0 = off and 5 = full speed)
// each click applies one count of the default motor resolution
byte MotorResolution = 50;
// true = forward, false =  reverse
boolean tankdirection = true;
// right motor speed
short rightmotorspd = 0;
// right motor direction true = forward, false =  reverse
boolean rightmotordir = true;
// left motor speed
short leftmotorspd = 0;
// left motor direction true = forward, false =  reverse
boolean leftmotordir = true;

// Laser servos
int LaserLR = 0;
int LaserUD = 1;

// Battery
short raw_read;
byte BatteryAverageCount = 0;
float BatteryVoltage = 0;
float BatteryAverageBuild = 0;
float BatteryAverageFinal = 0;

// Background
boolean Backgroundinit = true;
boolean BackgroundHearBeat = false;

// Secondary Communication
boolean Secondary_Ping_Sent = true;

// ISR vars
volatile int ISR_SecondaryComm = 0;
volatile int ISR_BatteryVoltage = 0;
volatile boolean ISR_LaserOn = false;
volatile int ISR_LaserTargetCount = 0;
volatile boolean ISR_LaserFire = false;
volatile int ISR_LaserFireLength = 0;
volatile int ISR_LaserFireCount = 0;
volatile byte Laserlrmov = 0;
volatile short Laserlrpos = LASECENTERLR;
volatile byte Laserudmov = 0;
volatile short Laserudpos = 600;
// the speed is in ms so multiple it by 100
// the ISR runs at 10 microseconds per interrupt
volatile int LaserTransitionSpeed = 1000;
volatile int ISR_LaserMoveCount = 0;

// WIFI objects
WiFiServer      webServer(webPort);
WiFiServer      socketServer(socketPort);
WebSocketServer webSocketServer;
WiFiClient      socketClient;

// laser servo objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

boolean on = false;

// remote control webpage, gzipped and base64 encoding to save space
char webpage_base64[] = "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";

void moveLaser(int pulseSize, int ServoNum)
{
  int pulseWidth;

  // Convert to pulse width
  pulseWidth = int(float(pulseSize) / 1000000 * SERVO_FREQUENCY * 4096);

  //Control Motor
  pwm.setPWM(ServoNum, 0, pulseWidth);
}

// set the motor direction
// Forward
// MRin1 = LOW      MLin1 = LOW
// MRin2 = HIGH     MLin2 = HIGH
// Reverse
// MRin1 = HIGH     MLin1 = HIGH
// MRin2 = LOW      MLin2 = LOW
void setMotorDirection(boolean tankdirection)
{
    if(tankdirection) {
        // Forward
        digitalWrite(MRin1, LOW);
        digitalWrite(MRin2, HIGH);
        digitalWrite(MLin1, LOW);
        digitalWrite(MLin2, HIGH);
    } else {
        // Reverse
        digitalWrite(MRin1, HIGH);
        digitalWrite(MRin2, LOW);
        digitalWrite(MLin1, HIGH);
        digitalWrite(MLin2, LOW);
    }
}

// set the motor speed
// if StopFlag = true stop the tank else set the speed as requested
void setMotorSpeed(boolean StopFlag = false)
{
    if(StopFlag) {
        analogWrite(MRen1, 0); // Motor 1 - Send PWM signal to L293D Enable pin
        analogWrite(MLen1, 0); // Motor 2 - Send PWM signal to L293D Enable pin
    } else {
        analogWrite(MRen1, rightmotorspd); // Motor 1 - Send PWM signal to L293D Enable pin
        analogWrite(MLen1, leftmotorspd); // Motor 2 - Send PWM signal to L293D Enable pin
    }
}

// chaeck and set the motor min/max ranges
void checkMotorRange()
{
    if(rightmotorspd > TankMaxSpeed) {
        rightmotorspd = TankMaxSpeed;
    }
    if(rightmotorspd < TankMinSpeed) {
        rightmotorspd = TankMinSpeed;   
    }
    
    if(leftmotorspd > TankMaxSpeed) {
        leftmotorspd = TankMaxSpeed;
    }
    if(leftmotorspd < TankMinSpeed) {
        leftmotorspd = TankMinSpeed;   
    }
}

void readBatteryStatus()
{ 
    // read the Battery Voltage
    raw_read = analogRead(VoltageMonitorPin);
    BatteryVoltage = (((float)raw_read) * 12.6 / 4095);
}

void sendBatteryStatus()
{ 
    // read the Battery Voltage
    readBatteryStatus();
    BatteryAverageBuild += BatteryVoltage;

    // make the average
    // average is the last 10 samples
    BatteryAverageCount++;
    if(BatteryAverageCount == 10) {
        BatteryAverageFinal = (BatteryAverageBuild / 10);
        BatteryAverageBuild = BatteryAverageCount = 0;
      
        // send it to the client
        webSocketServer.sendData("S:C = " + String(BatteryVoltage) + "v, A = " + String(BatteryAverageFinal) + "v, Raw = " + String(raw_read));

        // send an error message if the battery is below the error threshold
        if(BatteryAverageFinal <= LowVoltageCutoff) {
            // turn on Low Battery Light
            digitalWrite(LowVoltagePin, HIGH);
            webSocketServer.sendData("E:LOW BATTERY, Please Shout Down Now!");  
        } else {
            // turn off Low Battery Light
            digitalWrite(LowVoltagePin, LOW);
        }
    }
}

void printWifiStatus() 
{
    if(TerminalAttached) {
        Serial.print("SSID: "); Serial.println(WiFi.SSID());
        Serial.print("Signal strength (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
        Serial.print("IP address: "); Serial.println(WiFi.localIP());
        Serial.print("Gateway: "); Serial.println(WiFi.gatewayIP());
        Serial.print("Netmask: "); Serial.println(WiFi.subnetMask());
        Serial.print("Webpage is at http://"); Serial.print(WiFi.localIP()); Serial.println("/");
        Serial.print("Websocket is at http://"); Serial.print(WiFi.localIP()); Serial.println(":" + (String)socketPort + "/");
    }
}

void WiFiConnect() 
{
  while (WiFi.status() != WL_CONNECTED) 
  {
    if(TerminalAttached) {
        Serial.println("Connecting to " + (String)ssid + " ...");
    }
    WiFi.begin(ssid, pass);
    delay(5000);
  }
  
  IPAddress IP = WiFi.localIP();
  IP[3] = IPLastByte;
  
  WiFi.config(IP, WiFi.gatewayIP(), WiFi.gatewayIP(), WiFi.subnetMask());
  if(TerminalAttached) {
    Serial.println("Connected to " + (String)ssid);
  }
  
  webServer.begin();
  socketServer.begin();
  printWifiStatus();
  WiFi.lowPowerMode();
}

// Start MKR1010 software interrupt functions **********
void setup_timer4(uint16_t clk_div_, uint8_t count_)
{
   // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT8;          // Set the counter to 8-bit mode
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  REG_TC4_COUNT8_CC0 = count_;                      // Set the TC4 CC0 register to some arbitary value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_OVF;             // Enable TC4 interrupts

  uint16_t prescale=0;
  switch(clk_div_)
  {
    case 1:    prescale=TC_CTRLA_PRESCALER(0); break;
    case 2:    prescale=TC_CTRLA_PRESCALER(1); break;
    case 4:    prescale=TC_CTRLA_PRESCALER(2); break;
    case 8:    prescale=TC_CTRLA_PRESCALER(3); break;
    case 16:   prescale=TC_CTRLA_PRESCALER(4); break;
    case 64:   prescale=TC_CTRLA_PRESCALER(5); break;
    case 256:  prescale=TC_CTRLA_PRESCALER(6); break;
    case 1024: prescale=TC_CTRLA_PRESCALER(7); break;
  }
  REG_TC4_CTRLA |= prescale | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_ENABLE;    // Enable TC4
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
}
//----

uint16_t next_pow2(uint16_t v_)
{
  // the next power-of-2 of the value (if v_ is pow-of-2 returns v_)
  --v_;
  v_|=v_>>1;
  v_|=v_>>2;
  v_|=v_>>4;
  v_|=v_>>8;
  return v_+1;
}
//----

uint16_t get_clk_div(uint32_t freq_)
{
  float ideal_clk_div=48000000.0f/(256.0f*float(freq_));
  uint16_t clk_div=next_pow2(uint16_t(ceil(ideal_clk_div)));
  switch(clk_div)
  {
    case 32: clk_div=64; break;
    case 128: clk_div=256; break;
    case 512: clk_div=1024; break;
  }
  return clk_div;
}
//----

void setup_timer4(uint32_t freq_)
{
  uint16_t clk_div=get_clk_div(freq_);
  uint8_t clk_cnt=(48000000/clk_div)/freq_;
  setup_timer4(clk_div, clk_cnt);
}
// End MKR1010 software interrupt functions **********

// Interrupt Service Routine (ISR)
void TC4_Handler()
{
  if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)
  {
    // see if we are moving the Laser
    if(Laserlrmov != 0 || Laserudmov != 0) {
        ISR_LaserMoveCount++;

        if(ISR_LaserMoveCount == 1) {
            // move the laser left or right if it is on
            switch(Laserlrmov) {
                // move left
                case 1:
                    if(Laserlrpos < SERVO_MAX_PULSE_WIDTH) {
                        Laserlrpos++;
                    }
                    moveLaser(Laserlrpos, LaserLR);
                break;
                // move right
                case 2:
                    if(Laserlrpos > SERVO_MIN_PULSE_WIDTH) {
                        Laserlrpos--;
                    }
                    moveLaser(Laserlrpos, LaserLR);
                break;
            }

            // move the laser up or down if it is on
            switch(Laserudmov) {
                // move up
                case 1:
                    if(Laserudpos < LASEMAXUD) {
                        Laserudpos++;
                    }
                    moveLaser(Laserudpos, LaserUD);
                break;
                // move down
                case 2:
                    if(Laserudpos > SERVO_MIN_PULSE_WIDTH) {
                        Laserudpos--;
                    }
                    moveLaser(Laserudpos, LaserUD);
                break;
            }
        }

        // do this every x milliseconds
        // where x is set as the laser transition speed count
        if(ISR_LaserMoveCount >= LaserTransitionSpeed) {
            ISR_LaserMoveCount = 0;
        }
    }

    // see if Targeting is active and/or we were asked to Fire the Laser
    if(ISR_LaserFire) {
        // Fire
        // generate a digital pulse every 5 milliseconds
        // the length of the pulse is determined by the value of ISR_LaserFireLength (as milliseconds)
        if(ISR_LaserFireCount == 0) {
            digitalWrite(LASERDIGITALPWM, HIGH);
            ISR_LaserFireCount++;
        } else {
            ISR_LaserFireCount++;
            if(ISR_LaserFireCount == (((ISR_LaserFireLength + 1) * 100))) {
                digitalWrite(LASERDIGITALPWM, LOW);
                ISR_LaserFireCount = 0;
                ISR_LaserFire = false;
            }
        }
    } else if (ISR_LaserOn) {
        // Targeting
        // generate a 50 microsecond digital pulse every 5 milliseconds
        ISR_LaserTargetCount++;
        if(ISR_LaserTargetCount == 1) {
            digitalWrite(LASERDIGITALPWM, HIGH);
        }
        // 50 = (5 * 10 microseconds each interrupt)
        if(ISR_LaserTargetCount == 6) {
            digitalWrite(LASERDIGITALPWM, LOW);
        }
        // do this every 5 milliseconds
        if(ISR_LaserTargetCount == 500) {
            ISR_LaserTargetCount = 0;
        }
    }
    
    // see if it is time to send the battery status - every 3 seconds
    // if so tell the background to do it
    if(ISR_BatteryVoltage < 300000) {
      ISR_BatteryVoltage++;
    }

    // see if it is time to send the secondary communication status - every 5 seconds
    // if so tell the background to do it
    if(ISR_SecondaryComm < 500000) {
      ISR_SecondaryComm++;
    }
            
    // keep - clear interrupt
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;
  }
}

void setup()
{
    // we use this led to show the background is running
    // flased on and off every time we go through the background loop
    pinMode(LED_BUILTIN, OUTPUT);

    // setup the low voltage pin
    pinMode(LowVoltagePin, OUTPUT);
    digitalWrite(LowVoltagePin, LOW);

    // set the Target LED
    pinMode(TargetLED, OUTPUT);
    digitalWrite(TargetLED, LOW);
    
    // set the Fire LED
    pinMode(FireLED, OUTPUT);
    digitalWrite(FireLED, LOW);
   
    // Digital PWM Pin
    pinMode(LASERDIGITALPWM, OUTPUT);
    digitalWrite(LASERDIGITALPWM, LOW); // No PWM

    // set up the Motor Pins
    // Motor 1
    pinMode(MRen1, OUTPUT);
    pinMode(MRin1, OUTPUT);
    pinMode(MRin2, OUTPUT);
    // Motor 2
    pinMode(MLen1, OUTPUT);
    pinMode(MLin1, OUTPUT);
    pinMode(MLin2, OUTPUT);
    // set the direction
    setMotorDirection(tankdirection);

    // Serial port initialization
    if(TerminalAttached) {
        Serial.begin(57600);
    }

    // Get the Initial Battery Status so we can preset the battery average until we have one (every 30 seconds)
    // also check and make sure we are good to go else set the Low Voltage LED and wait
    analogReadResolution(12);
    while(BatteryAverageFinal <= LowVoltageCutoff) {
      readBatteryStatus();

      // check to make sure we got enough battery to continue
      if(BatteryVoltage <= LowVoltageCutoff) {
        // turn on Low Battery Light and do nothing else
        digitalWrite(LowVoltagePin, HIGH);
      } else {
          BatteryAverageFinal = BatteryVoltage;
          digitalWrite(LowVoltagePin, LOW);
      }

      // delay 10 second, remember we have to call sendBatteryStatus() 10 times to get the average so 1000 msec * 10 = 10 seconds
      delay(1000);
    }

    // Start up the laser servos
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQUENCY);

    // set left/right laser servo to center
    // (2350 - 650 = 1,700 / 2 = 850 + 650 = 1500
    moveLaser(Laserlrpos, LaserLR);
    // set up/down laser servo to down
    moveLaser(Laserudpos, LaserUD);

    if(TerminalAttached) {
        Serial.println("\nStart MultiServers");
        Serial.println("Version " + String(WIFININA_GENERIC_VERSION));
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    {
        if(TerminalAttached) {
            Serial.print("Your current firmware NINA FW v");
            Serial.println(fv);
            Serial.print("Please upgrade the firmware to NINA FW v");
            Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
        }
    }

    // Secondary Serial port initialization
    Serial1.begin(57600); // Initialize serial port to send and receive at 57600 baud
    // wait until it is open
    while (!Serial1) {  
    }

    // done initilization so start the interrupts
    // call ISR - TC4_Handler 100000 times per second
    // an interrupt is called every 10 microseconds so to get:
    // count 1 interrupts = 10us
    // count 5 interrupts = 50us
    // count 100 interrupts = 1ms
    // count 500 interrupts = 5ms
    // count 1000 interrupts = 10ms
    // count 300000 interrupts = 3s
    // count 500000 interrupts = 5s
    setup_timer4(100000);

    // setup complete
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
    if(WiFi.status() != WL_CONNECTED)
    {
        if(TerminalAttached) {
            Serial.println("Lost WiFi connection");
        }
        WiFi.end();
        WiFiConnect();
    }

    WiFiClient webClient = webServer.available();
  
    if(webClient.connected())
    {
        if(TerminalAttached) {  
            Serial.print("New client: "); Serial.print(webClient.remoteIP()); Serial.print(":"); Serial.println(webClient.remotePort());
        }
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
            if(TerminalAttached) {  
                Serial.print(header.substring(0, header.length() - 1));
            }
          
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
                if(TerminalAttached) {
                    Serial.println("--Interface webpage sent");
                }
            } 
            else 
            {
                webClient.println("HTTP/1.1 404 Not Found\nContent-Type: text/plain; charset=utf-8\n\n404 Not Found\n");
                if(TerminalAttached) {
                    Serial.println("--Page not found");
                }
            }
          
            webClient.stop();
            if(TerminalAttached) {
                Serial.println("--Client disconnected");
            }
        }
      }
    }

  if(!socketClient.connected()) 
  {
    socketClient = socketServer.available();
    
    if (socketClient.connected() && webSocketServer.handshake(socketClient)) 
    {
        if(TerminalAttached) {
            Serial.print("\n--Websocket connected to: ");
            Serial.print(socketClient.remoteIP());
            Serial.print(":");
            Serial.println(socketClient.remotePort());
        }
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
        if(TerminalAttached) {
            Serial.println("Websocket Flushed");
        }

        // flush the serial1 port - Arduion to Arduino comms
        while(Serial1.available()) {
            char ch = Serial1.read();
        }
        if(TerminalAttached) {
            Serial.println("Serial Port 1 Flushed");
            Serial.println("Background Init Complete");
        }
    }
    
    // Background Process 1
    // see if we have a command/request from the user 
    String data = webSocketServer.getData();
    if (data.length() > 0) 
    {
      String cmd = data.substring(0, data.indexOf(":"));
      String setting = data.substring(data.indexOf(":") + 1);
      boolean goodCommand = true;

      // process command
      switch (cmd.toInt()) {
        case 1:
            // Forward
            if(tankdirection) {
                // first see if one is faster then the other if so make them the same else increase both speeds
                // make both motors the same speed, which is faster
                if(rightmotorspd > leftmotorspd) {
                    // set the left to the right
                    leftmotorspd = rightmotorspd;
                } else if (leftmotorspd > rightmotorspd) {
                    // set the right to the left
                    rightmotorspd = leftmotorspd;
                } else {
                    rightmotorspd += MotorResolution;
                    leftmotorspd += MotorResolution;
                    checkMotorRange();                    
                }
            } else {
                // first stop the tank
                setMotorSpeed(true);
                delay(TankSwitchTime);

                // second set the direction to forward
                tankdirection = true;
                setMotorDirection(tankdirection);
                rightmotorspd = TankMinSpeed;    
                leftmotorspd = TankMinSpeed;    
            }

            setMotorSpeed();
            break;
        case 2:
            // Reverse
            if(!tankdirection) {
                // first see if one is faster then the other if so make them the same else increase both speeds
                // make both motors the same speed, which is faster
                if(rightmotorspd > leftmotorspd) {
                    // set the left to the right
                    leftmotorspd = rightmotorspd;
                } else if (leftmotorspd > rightmotorspd) {
                    // set the right to the left
                    rightmotorspd = leftmotorspd;
                } else {
                    rightmotorspd += MotorResolution;
                    leftmotorspd += MotorResolution;
                    checkMotorRange(); 
                }               
             } else {
                // first stop the tank
                setMotorSpeed(true);
                delay(TankSwitchTime);

                // second set the direction to reverse
                tankdirection = false;
                setMotorDirection(tankdirection);
                rightmotorspd = TankMinSpeed;    
                leftmotorspd = TankMinSpeed;    
            }
            
            setMotorSpeed();
            break;
        case 3:
            // Max Forward - both motors on
            if(tankdirection) {
                rightmotorspd = TankMaxSpeed;
                leftmotorspd = TankMaxSpeed;
            } else {
                // first stop the tank
                setMotorSpeed(true);
                delay(TankSwitchTime);

                // second set the direction to forward
                tankdirection = true;
                setMotorDirection(tankdirection);
                rightmotorspd = TankMaxSpeed;    
                leftmotorspd = TankMaxSpeed;    
            }
            
            setMotorSpeed();
            break;
        case 4:
            // Stop both motors
            rightmotorspd  = 0;
            leftmotorspd  = 0;
            setMotorSpeed();
            break;
        case 5:
            // Max Reverse - both motors on
            if(!tankdirection) {
                rightmotorspd = TankMaxSpeed;
                leftmotorspd = TankMaxSpeed;
            } else {
                // first stop the tank
                setMotorSpeed(true);
                delay(TankSwitchTime);

                // second set the direction to reverse
                tankdirection = false;
                setMotorDirection(tankdirection);
                rightmotorspd = TankMaxSpeed;    
                leftmotorspd = TankMaxSpeed;    
            }
            
            setMotorSpeed();
            break;
        case 6:
            // Left so:
            // apply more power to the right motor and less to the left motor
            rightmotorspd += MotorResolution;
            if(rightmotorspd > TankMaxSpeed) {
                rightmotorspd = TankMaxSpeed;
            }
            // apply less power to the left motor
            if(leftmotorspd >= TankMinSpeed) {
                leftmotorspd -= MotorResolution;
            } else {
                leftmotorspd = 0;    
            }

            setMotorSpeed();
            break;
        case 7:
            // Right so:
            // apply more power to the left motor and less to the right motor
            leftmotorspd += MotorResolution;
            if(leftmotorspd > TankMaxSpeed) {
                leftmotorspd = TankMaxSpeed;
            }
            // apply less power to the left motor
            if(rightmotorspd >= TankMinSpeed) {
                rightmotorspd -= MotorResolution;
            } else {
                rightmotorspd = 0;    
            }

            setMotorSpeed();
            break;
        case 8:
            // Max Left so:
            // Max power right right motor
            // 0 Power the left motor
            rightmotorspd = TankMaxSpeed;
            leftmotorspd = 0;
            if(!tankdirection) {
                // first stop the tank
                setMotorSpeed(true);
                delay(TankSwitchTime);

                // second set the direction to forward
                tankdirection = true;
                setMotorDirection(tankdirection);
            }

            setMotorSpeed();
            break;
        case 9:
            // Straight weather the tank is going forward or reverse
            // if they motors are not the same speed
            if(rightmotorspd != leftmotorspd) {
                // if one motor is positive then find the highest power and set the other to it
                if(rightmotorspd > leftmotorspd) {
                    // set the left to the right
                    leftmotorspd = rightmotorspd;
                } else {
                    // set the right to the left
                    rightmotorspd = leftmotorspd;
                }
            }

            setMotorSpeed();
            break;
        case 10:
            // Max Right so:
            // Max power right left motor
            // 0 Power the right motor
            rightmotorspd = 0;
            leftmotorspd = TankMaxSpeed;
            if(!tankdirection) {
                // first stop the tank
                setMotorSpeed(true);
                delay(TankSwitchTime);

                // second set the direction to forward
                tankdirection = true;
                setMotorDirection(tankdirection);
            }

            setMotorSpeed();
            break;
        case 11:
            // Target LED to on
            ISR_LaserOn = !ISR_LaserOn;
            if(ISR_LaserOn) {
                digitalWrite(TargetLED, HIGH);
                delay(100);
                digitalWrite(TargetLED, LOW);
            } else {
                // make sure the laser is always off if not targeting
                delay(10);
                digitalWrite(LASERDIGITALPWM, LOW);
            }
            break;
        case 12:
            // Laser Left:
            if(Laserlrmov == 1) {
                Laserlrmov = 0;
            } else {
                Laserlrmov = 1;
            }
            break;
        case 13:
            // Laser Right:
            if(Laserlrmov == 2) {
                Laserlrmov = 0;
            } else {
                Laserlrmov = 2;
            }
            break;
        case 14:
            // Laser Up:
            if(Laserudmov == 1) {
                Laserudmov = 0;
            } else {
                Laserudmov = 1;
            }
            break;
        case 15:
            // Laser Down:
            if(Laserudmov == 2) {
                Laserudmov = 0;
            } else {
                Laserudmov = 2;
            }
            break;
        case 16:
            // Fire Set the Fire LED to on for user defined time
            if(ISR_LaserFireLength > 0) {
                ISR_LaserFire = true;
                ISR_LaserFireCount = 0;
            }
            digitalWrite(FireLED, HIGH);
            delay(100);
            digitalWrite(FireLED, LOW);
            break;
        case 17:
            // Send request/command to secondary processor
            if(setting.length() > 0) {
                for (byte i = 0; i < setting.length(); i++) {
                    Serial1.write(setting[i]);
                }
                // send EOC
                Serial1.write('.');
            }
            break;
        case 18:
            // this is the -Battery+ button
            // no need to do anything just send the the resonse in the "process response"
            break;
        case 19:
            // process the command
            goodCommand = false;
              
            if(setting.charAt(0) == 'L') {
                String laserlength = setting.substring(1);
                ISR_LaserFireLength = (laserlength.toInt());
                webSocketServer.sendData("L:" + String(ISR_LaserFireLength) + " ms");
            } else if (setting.charAt(0) == 'M') {
                String motorlength = setting.substring(1);
                MotorResolution = (motorlength.toInt());
                webSocketServer.sendData("M:" + String(MotorResolution) + " inc");
            } else if (setting.charAt(0) == 'T') {
                String lasertransition = setting.substring(1);
                // the speed is in ms so multiple it by 100
                // the ISR runs at 10 microseconds per interrupt
                LaserTransitionSpeed = ((lasertransition.toInt()) * 100);
                webSocketServer.sendData("T:" + String((LaserTransitionSpeed / 100)) + " ms");
            } else if (setting.charAt(0) == 'D') {
                webSocketServer.sendData("D:tankdirection: " + String(tankdirection));   
                webSocketServer.sendData("D:rightmotorspd: " + String(rightmotorspd));   
                webSocketServer.sendData("D:leftmotorspd: " + String(leftmotorspd));
                webSocketServer.sendData("D:EndDebug:");
            }
            break;
        case 20:
            // SpeedUP
            rightmotorspd += MotorResolution;
            leftmotorspd += MotorResolution;
            checkMotorRange();

            setMotorSpeed();
            break;
        case 21:
            // SlowDown
            rightmotorspd -= MotorResolution;
            if(rightmotorspd < TankMinSpeed) {
                rightmotorspd = 0;
            }

            leftmotorspd -= MotorResolution;
            if(leftmotorspd < TankMinSpeed) {
                leftmotorspd = 0;
            }

            setMotorSpeed();
           break;
        default:
            goodCommand = false;
            webSocketServer.sendData("E:" + cmd + " - " + setting);
            break;
      }

      // process response
      if(goodCommand == true) {
        switch (cmd.toInt()) {
            // targeting
            case 11:
                webSocketServer.sendData("R:Laser Targeting");
                break;
            // laser left - right
            case 12:
            case 13:
                break;
            // laser up - down
            case 14:
            case 15:
                break;
            // fire laser
            case 16:
                webSocketServer.sendData("R:Laser Fired");
                break;
            // serial communication
            case 17:
                webSocketServer.sendData("R:Secondary TX " + String(setting));
                break;
            case 18:
                webSocketServer.sendData("R:CBattery = " + String(BatteryVoltage) + "V, ABattery = " + String(BatteryAverageFinal) + "V, Raw = " + String(raw_read));
                break;
            // motor movements
            default:
                float LMpercent = ((float)leftmotorspd /  (float)TankMaxSpeed) * 100.0;
                float RMpercent = ((float)rightmotorspd /  (float)TankMaxSpeed) * 100.0;
                webSocketServer.sendData("R:leftmotorspd " + String(LMpercent) + "%" + " , rightmotorspd " + String(RMpercent) + "%");
                break;
          }
      }
    }

    // Background Process 2
    // see if it's time to send the battery status
    // the function will send a status update every 10 times we call it so we send the status every 30 seconds
    if(ISR_BatteryVoltage == 300000) {
      ISR_BatteryVoltage = 0;
      sendBatteryStatus();
    }

    // Background Process 3
    // see if it's time to pig the secondary processor
    // the function will send a secondary communication ping every 5 seconds
    if(ISR_SecondaryComm == 500000) {
        // if the flag is still true we did not get a response and secondary is disconnected
        if(Secondary_Ping_Sent == true) {
            webSocketServer.sendData("P:Disconnected/Timeout");
        } else {
            webSocketServer.sendData("P:Connected");
        }

        // send the a ping to the secondary
        // we expect a < back
        Serial1.write('>');
        Serial1.write('.');
        Secondary_Ping_Sent = true;
        ISR_SecondaryComm = 0;
    }

    // Background Process 4
    // communicate with peripheral Arduino - all the time 
    if(Serial1.available()) { // Check to see if at least one character is available
        String responsefrom = Serial1.readStringUntil('.');
      
        if(responsefrom.charAt(0) == '<') {
            Secondary_Ping_Sent = false;
        } else if(responsefrom.charAt(0) == '>'){
            webSocketServer.sendData("R:Loopback/Off");
        } else if(responsefrom.charAt(0) == 'A'){
            webSocketServer.sendData("R:Secondary Complete");    
        } else {
            webSocketServer.sendData("R:Secondary Bad Response");
        }
    }
  }

    // Background Process 5
    // show the background heart
    BackgroundHearBeat = !BackgroundHearBeat;
    if(BackgroundHearBeat) {
        digitalWrite(LED_BUILTIN, LOW);  
    } else {
        digitalWrite(LED_BUILTIN, HIGH);  
    }
}
