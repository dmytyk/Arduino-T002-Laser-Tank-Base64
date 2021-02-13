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
> - [Network Parameters](/network_parameters.h) - file to store your WiFi network information
> - [Arduino Code](/T002_LaserTankBase64.ino) - Arduino code segment to interact with the webpage
> - [7ZIP](https://www.7-zip.org/) - free tool to create a gzip file from our webpage
> - [Base64 Guru](https://base64.guru/) - super-de-duper website to Base64 encode our gzipped file 

### The Walk Through

The following steps walk you through the process of zipping, encoding, saving, sending and running the webpage

1. Download and install 7ZIP
2. Connect the MKR1010 to your computer, create a folder (T002_LaserTankBase64) save the webpage.html, network_parameters.h and the T002_LaserTankBase64.ino files
3. Ensure you change the ***MYHOME_SSID***, ***MYHOME_PASSWORD*** to match your network and set the ***IPAaddress*** you want to use  
4. Open the T002_LaserTankBase64 folder you created
5. Right click on webpage.html, select 7-Zip, then Add Archive..., you should have a screen that looks like the one below.  The default parameters and file name are good for out example

![7zip](/Images/7zip.JPG)

6. Click Okay to create the file webpage.html.gz
7. Go to the [Base64 Guru](https://base64.guru/) website, select File to Base64 from Encoders menu on the left, your screen should show

![Base64](/Images/Base64.JPG)

8. Go to Local File and upload your zipped file webpage.html.gz, ensure you have "Output Format" of Plain text -- just the Base64 value selected, then click the Encode file to Base64 button. You get the following output in the Base64 window

![Base64Output](/Images/Base64Output.JPG)

9. Click the small copy button in the upper right corner
10. Open your T002_LaserTankBase64.ino file, navigate to line where you see ***char webpage_base64[] = "";***, place your cursor in between the double quotes and paste the Base64 text you copied in step 9
11. Save the file then click the Upload Arrow to load the code to the MKR1010
12. After you successfully loaded the MKR1010, open up a browser and enter the IP address you selected in the [Network Parameters](/network_parameters.h) file
13. That's it you can now interact with the Arduino hardware, you should see the following webapge

![LT WebPage Image](/Images/LaserTankWebPage.JPG)

Once again this tutorial will not dive into the webpage or Arduino code workings they are being provided to allow us to demonstrate using 7Zip and Base64 encoding, we will discuss the webpage and Arduino code in future tutorials.  Our goal was to show you to use 7Zip and Base64 encoding to reduce the size of your webpage and save valuable Arduino code space.  You can watch this tutorial at:

[T002 Laser Tank Base64 Web Page](https://www.youtube.com/watch?v=oKCXiYc311A&list=PLVApLwWluBD4ELsZ1-4vU5UWpnnuXdoNG&index=2)

## Next Up
> - Tutorials on Software Interrupts, Background Processing, Board to Board communication, Pulse Simulation and other Fun Stuff

Follow me on [YouTube](https://www.youtube.com/channel/UClwcP7ByE6Ia9DmKfP0C-UQ) to catch the "Next Up" stuff and thanks for Hanging Out at the Shack!

