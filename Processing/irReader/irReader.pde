import processing.serial.*;
Serial myPort;
float value;

void setup() {
  size(400, 400);
  myPort = new Serial(this, Serial.list()[8], 9600);
}

void draw() {
	background(#393939);
	rectMode(CENTER);

  pushStyle();
    fill(#24df95);
    noStroke();
    rect(width/2, height/2, width*0.7, map(value, 15, 200, 0, height));
  popStyle();

  fill(#000000);
  textAlign(CENTER);
  text("("+floor(value) + " cm)", width/2, height/2);
}

void serialEvent(Serial p) {
  // get message till line break (ASCII > 13)
  String message = myPort.readStringUntil(13);
  if(message != null){
    value = float(message);
    println(value);
  }
}