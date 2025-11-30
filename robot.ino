
//Libraries
#include <ESP8266WiFi.h>  
#include "24s_PubSubClient.h"
#include "24s_WiFiManager.h"  
#include <Servo.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include "SSD1306.h"
#include <Math.h>

//Defining Sensor Distances
int distance_middle;
int distance_right;
int distance_left;

//Pin Definitions 
Ultrasonic ultrasonic_middle(D8, D5); //Trigger, Echo
Ultrasonic ultrasonic_right(D9, D6); //Trigger, Echo
Ultrasonic ultrasonic_left(D10, D7); //Trigger, Echo

SSD1306 display(0x3C,D14,D15); //Ox#C - I2C address
                               //D14 (SDA/Serial Data)
                               //D15 (SCL/Serial Clock)

#define rightmotorpin D0 //0 max speed, 90 stop
#define leftmotorpin D2  //180 max speed, 90 stop
Servo rightmotor; //Create servo motor1 object to control a servo
Servo leftmotor; //Create servo motor2 object to control a servo

//Start of Movement Function Section

//Correction Functions
void correctionL() { //Adjusts robot if it gets too close to a wall via left sensor
  rightmotor.write(82);
  leftmotor.write(116);
  delay(300);
}

void correctionR() { //Adjusts robot if it gets too close to a wall via right sensor
  rightmotor.write(64);
  leftmotor.write(98);
  delay(300);
}
//Forwards & Backwards Functions

void backwards() { //Backwards Movement
  rightmotor.write(105); 
  leftmotor.write(75);
  delay(800);
}

void forwards() { //Forwards Movement
rightmotor.write(79);
leftmotor.write(106);
delay(200);
}

void forwardsSlow() { //Slow Down Function when near Target
  rightmotor.write(82);
  leftmotor.write(103);
  delay(200);
}

//Turning Functions
void turnleft90() { //90 Degree Left turn
  leftmotor.write(90);
  rightmotor.write(45);
  delay(600);
}

void turnright90() { //90 Degree Right Turn
  leftmotor.write(110);
  rightmotor.write(90);
  delay(600);
}

void turnleft45() { //45 Degree Left Turn
  leftmotor.write(90);
  rightmotor.write(64);
  delay(200);
}

void turnright45() { //45 Degree Right Turn
  leftmotor.write(116);
  rightmotor.write(90);
  delay(200);
} 

void turnleft20() { //20 Degree Left Turn
  leftmotor.write(116);
  rightmotor.write(90);
  delay(100);
}

void turnright20() { //20 Degree Right Turn
  leftmotor.write(90);
  rightmotor.write(64);
  delay(100);
}

//Stop Functions
void stop() { //Full Stop
  leftmotor.write(90);
  rightmotor.write(90);
  delay(200);
}

void targetstop() { //2 Second Stop at Target
  leftmotor.write(90);
  rightmotor.write(90);
  delay(2000);
}

void fullstop() { //End Stop when at Target 4
  leftmotor.write(90);
  rightmotor.write(90);
  delay(10000);
}

//End of Movement Function Section
  
//MQTT Communication associated variables
char payload_global[100];                     
boolean flag_payload;                         

//MQTT Setting variables  
const char* mqtt_server= "192.168.0.223"; // South
//const char* mqtt_server= "192.168.0.140"; // North          //MQTT Broker(Server) Address
const char* MQusername = "user";               //MQTT username
const char* MQpassword = "Stevens1870";               //MQTT password
const char* MQtopic    = "louis_lidar_new";               //MQTT Topic (Arena I/II)
const int mqtt_port    = 1883;          //MQTT TCP/IP port number 

//WiFi Setting variables
const char* ssid     = "TP-Link_4DB1";  //South        //Wi-Fi SSID (Service Set IDentifier)   
const char* password = "01747331";     // South
//const char* ssid     = "TP-Link_9402";     // North            //Wi-Fi SSID (Service Set IDentifier)   
//const char* password = "77556578";        // North       //Wi-Fi Password

//WiFi Define
WiFiClient espClient;                         
PubSubClient client(espClient);             
      
void setup_wifi() { 
  delay(10);
  // We start by connecting to a Stevens WiFi network
  WiFi.begin(ssid, password);           
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");                        
  }
  randomSeed(micros());                       
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    payload_global[i] = (char)payload[i];
  }
  payload_global[length] = '\0';              
  flag_payload = true;                        
}

void reconnect() {                                                                
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-";       
    clientId += String(random(0xffff), HEX);  
    // Attempt to connect                     
    if (client.connect(clientId.c_str(),MQusername,MQpassword)) {
      client.subscribe(MQtopic);             
    } else {
      // Wait 5 seconds before retrying
      Serial.println("Reconnecting");
      delay(5000);
    }
  }
}

void setup() {
  
  Serial.begin(115200);

  //Motors
  rightmotor.attach(rightmotorpin); //motor1 is attached using the motor1pin
  leftmotor.attach(leftmotorpin); //motor2 is attached using the motor2pin

  //MQTT
  setup_wifi();                               
  delay(3000);
  Serial.println("Wemos POWERING UP ......... ");
  client.setServer(mqtt_server, mqtt_port);          //This 1883 is a TCP/IP port number for MQTT 
  client.setCallback(callback); 

  //oLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.display();

  //Movement out of Starting Point
  forwards();
  forwards();
  forwards();
  forwards();
  forwards();
  forwards();
  turnleft90();
}
  //Intializing Current and Past Position
  int currx = 0;
  int curry = 0;

  int pastx = 0;
  int pasty = 0;

  //Track Targets 
  int currTarget = 0; 

void loop() {
  
  //Defining distances as readings from the Ultrasonic Sensors
  distance_middle = ultrasonic_middle.read(CM);
  distance_right = ultrasonic_right.read(CM);
  distance_left = ultrasonic_left.read(CM);

  //Subscribe the data from MQTT server
  if (!client.connected()) {
    Serial.print("...");
    reconnect();
  }

  client.loop();                              
  
  String payload(payload_global);              
  int testCollector[10];                      
  int count = 0;
  int prevIndex, delimIndex;
    
  prevIndex = payload.indexOf('[');           
  while( (delimIndex = payload.indexOf(',', prevIndex +1) ) != -1){
    testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
    prevIndex = delimIndex;
  }
  delimIndex = payload.indexOf(']');
  testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();

  //Past Location of Robot
  int pastx = currx;
  int pasty = curry;
 
  //Robot location x,y from MQTT subscription variable testCollector 
  currx = testCollector[0];
  curry = testCollector[1];

  //Math Calculations for Pathfinding Logic
  int target_x_locations[] = {1600,2000,110,700}; //Array for X-Coords of Targets
  int target_y_locations[] = {130,700,150,130}; //Array for Y-Coords of Targets

  int distanceT = sqrt(sq(target_x_locations[currTarget] - currx) + sq(target_y_locations[currTarget] - curry)); // Distance from Robot to Target
  
  int v1[2] = {(currx - pastx), (curry - pasty)};   //Vector of Robot
  int v2[2] = {target_x_locations[currTarget] - currx, target_y_locations[currTarget] - curry};  // Vector to the target
  
  int dot_product = v1[0] * v2[0] + v1[1] * v2[1]; //Calculate the dot product
  
  float magnitude_v1 = sqrt(pow(v1[0], 2) + pow(v1[1], 2)); //Calculate the magnitudes of Vector 1 
  float magnitude_v2 = sqrt(pow(v2[0], 2) + pow(v2[1], 2)); //Calculate the magnitudes of Vector 2
  
  float cos_theta = dot_product / (magnitude_v1 * magnitude_v2); //Calculate the cosine of the angle
  
  float theta_radians = acos(cos_theta); //Calculate the angle in radians
  
  float theta_degrees = theta_radians * (180.0 / M_PI); //Convert radians to degrees

  int cross_product = v1[0] * v2[1] - v1[1] * v2[0]; //Calculate the cross product to determine the sign of the angle
  int sign = cross_product >= 0 ? 1 : -1; //Positive or negative sign

  theta_degrees = theta_degrees * sign; //Final Angle
 
  //Stopping at Target, Updating to next Target & Displaying coords on oLED
  if(currx >= target_x_locations[currTarget] - 50 && currx <= target_x_locations[currTarget] + 50 && curry >= target_y_locations[currTarget] - 50 && curry <= target_y_locations[currTarget] + 50) {
    currTarget = currTarget + 1;
    targetstop();
    display.clear();
    display.drawString(0,0, String(currx));
    display.drawString(0,16, String(curry));
    display.display();
    display.drawString(0,32,"Target ");
    display.drawString(0,48, "Located");
    display.display();
    display.clear();
    delay(100);
  }

  //Slowing Down Near Target
  if(distanceT < 450) {
    forwardsSlow();
  }

  //Stopping Fully after reaching Final Target
  if(currTarget == 4){
    Serial.println("stopping at last");
    fullstop();
  }

  //Path-Finding Logic, + Angle Turn Left, - Angle Turn Right 
  if(distance_left > 17 && distance_right > 17 && distance_middle > 14) {
    if (theta_degrees >= -80 && theta_degrees <= 80) {
      if(theta_degrees >= -40 && theta_degrees <= 40) {
        forwards();
      }
      else if(theta_degrees <= -40) {
        turnright20();
      }
      else if (theta_degrees >= 40) {
        turnleft20();
      }
    }
    else if (theta_degrees <= -80) {
        turnright45();
    }
    else if (theta_degrees >= 80) {
        turnleft45();
    }
    forwards();
  }
  
  //Obstacle Avoidance Logic 
  if (distance_middle <= 14) {
    backwards();
    stop();
    turnright45();
    if(distance_left <= 17 && distance_middle <= 14) {
      backwards();
      stop();
      turnright90();
      turnright90();
    } 
    else {
      stop();
      turnleft90();
    }
  }
  if (distance_right <= 17){
    correctionR();
    delay(200);
    }
  if (distance_left <= 17) {
    correctionL();
    delay(200);
  }

  //oLED Display of Current Coords and Current Target
  display.drawString(0,0, "Current Coords");
  display.drawString(0,16, String(currx));
  display.display();
  display.drawString(0,32, "Current Target");
  display.drawString(0,48, String(currTarget));
  display.display();
  display.drawString(60, 16, String(curry));
  display.display();
  display.clear();
 
}
