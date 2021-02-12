# Arduino-T002-Laser-Tank-Base64

### Laser Tank Base64 Tutorial
This is tutorial 2 for my 5.5w Laser Tank (LT).  It demonstrates how to incorporate Base64 and gzip compression to reduce the size of your Arduino code base.  You create a custom webpage, gzip it, encode it with Base64 and save it to your Arduino.  After a connection is made your Arduino web server will decode it then transmit it to the client.  The client can then interact with the Arduino hardware using the integrated websocket server.

Below is the webpage for the LT.  We will not be covering its functionality at this time (future tutorials), our goal is to demonstrate how to compress, store and run the page. 

### WebPage

> #### Features
> - Multiple Buttons to control speed and direction of the LT
> - Ability to change the LT's configuration
> - Ability to control the Secondary Processor
> - Provide Status and System Feedback
> - Uses HTML to generate the screen layout & buttons
> - Uses javascript to create the clients web & socket server's used for communications with the Arduino's web and socket server's

![LT WebPage Image](/Images/LaserTankWebPage.JPG)


> #### Things we need to *Getter Done*
> - [7ZIP](https://www.7-zip.org/) - free tool to create a gzip file from our webpage
> - [Base64 Guru](https://base64.guru/) - super-de-duper website to Base64 encode our gzipped file 
> - [LT WebPage Code](/webpage.html) - the webpage we are using
> - Arduino code segment to interact with the webpage.  Below is the Arduino code segment from the Laser Tank that stores the webpage, transmits it to the client when a connection is made then interacts with it.





