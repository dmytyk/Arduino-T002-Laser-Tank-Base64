# Arduino-T002-Laser-Tank-Base64

### Laser Tank Base64 Tutorial
This is tutorial number 2 for my 5.5w Laser Tank (LT). I wanted the ability to control the LT from my desktop (during development) and/or my phone/tablet (actually using it).

![Overview](/Images/OverView.jpg)

I required a robust page with lots of features, however that can take up a lot of valuable code space.  This tutorial demonstrates how to incorporate Base64 and gzip compression to reduce the size of your Arduino code base. The process is straight forward, you create a custom webpage, gzip it, encode it with Base64 and save it to your Arduino.  After a connection is made your Arduino web server will decode it then transmit it to the client.  The client can then interact with the Arduino hardware using the integrated websocket server.

### The WebPage

> #### Features
> - Multiple Buttons to control the speed and direction of the LT including the Laser
> - Provide Status and System Feedback
> - Ability to change the LT's configuration
> - Ability to control the Secondary Processor
> - Uses HTML to generate the screen layout & buttons
> - Uses javascript to create the clients web & socket server's used for communications with the Arduino's web and socket server's

Below is a screenshot of the rendered webpage for the LT.  We will not be covering its functionality at this time (future tutorials), our goal is to demonstrate how to compress, store and run the page. 

![LT WebPage Image](/Images/LaserTankWebPage.JPG)

> #### Things we need to *Getter Done*
> - [Arduino MKR1010](https://store.arduino.cc/usa/mkr-wifi-1010) - Arduino microcontroller we are using
> - [LT WebPage Code](/webpage.html) - the webpage we are using
> - [Arduino Code](/T002_LaserTankBase64.ino) - Arduino code segment to interact with the webpage
> - [7ZIP](https://www.7-zip.org/) - free tool to create a gzip file from our webpage
> - [Base64 Guru](https://base64.guru/) - super-de-duper website to Base64 encode our gzipped file 





