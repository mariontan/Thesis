/**
 * Simple Read
 * 
 * Read data from the serial port and change the color of a rectangle
 * when a switch connected to a Wiring or Arduino board is pressed and released.
 * This example works with the Wiring / Arduino program that follows below.
 */

import processing.serial.*;

Serial myPort;  // Create object from Serial class
String val;      // Data received from the serial port
PrintWriter output;
String timestamp;

void setup() 
{
  size(640, 360);
  // Create the font
  //textFont(createFont("Georgia", 36));
  myPort = new Serial(this, "COM6", 115200);
  timestamp = year() + nf(month(),2) + nf(day(),2) + "-"  + nf(hour(),2) + nf(minute(),2) + nf(second(),2);
  println(timestamp);
  output = createWriter( "noMovement"+timestamp+".csv" );
}

void draw()
{
  if ( myPort.available() > 0) {  // If data is available,
    val = myPort.readStringUntil('\n');         // read it and store it in val
  }
  //catches the null 
  if(val == null){
     println("wait until null disappears");
  }
  else{
    printTime();
    output.println( val );
  }
   
}

void keyReleased(){
}

void printTime(){
  println(val);
  textSize(24);
  clear();
  text(val,100,50);
  fill(0,0,255);
}
/******to send int from processing to arduino**********/
/*
convert int to string in processing
then parse string back to int 
*/