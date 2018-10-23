//include the library code:
#include <event_timer.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_CC3000.h>

unsigned long deliveryStartTime = 0;
volatile unsigned long deliveryDistance = 0;

#define __DEBUG__

///////////////////////////////////////////
//CC3000 setup
///////////////////////////////////////////
#define ADAFRUIT_CC3000_IRQ   3 // MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT  4
#define ADAFRUIT_CC3000_CS    10

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                         SPI_CLOCK_DIVIDER);

#define WLAN_SSID       "MOTOROLA-4A7AC"        // cannot be longer than 32 characters!
#define WLAN_PASS       "6a46f4a354ae9fd8c031"

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_UNSEC

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
// received before closing the connection.  If you know the server
// you're accessing is quick to respond, you can reduce this value.

const char destServer[] = "ec2-34-209-142-24.us-west-2.compute.amazonaws.com";

//destinations (which double as states)
#define WAITING_FOR_ORDER 0x00
#define HQ 0x60
#define WAREHOUSE_A 0x3A
#define WAREHOUSE_B 0x3B
#define WAREHOUSE_C 0x3C
#define HOUSE_1 0x41
#define HOUSE_2 0x42
#define HOUSE_3 0x43
#define HOUSE_4 0x44
#define HOUSE_5 0x45

//roads
#define ROAD_Q   0x51
#define ROAD_W   0x52
#define ROAD_BC  0x53
#define ROAD_H   0x54
#define ROAD_345 0x55

#define ROAD_A 0x1A
#define ROAD_B 0x1B
#define ROAD_C 0x1C

#define ROAD_1 0x21
#define ROAD_2 0x22
#define ROAD_3 0x23
#define ROAD_4 0x24
#define ROAD_5 0x25

#define DARK 1
#define LIGHT 0

byte state = WAITING_FOR_ORDER;
byte destination = HQ;

//motor control parameters
#define BASE_SPEED 200
#define SPEED_ADJUST 1

#define OUT_R1 A3
#define OUT_R2 A4
#define SPEED_R 5

#define OUT_L1 A1
#define OUT_L2 A2
#define SPEED_L 6

//direction defines
#define LEFT_FORWARD1 0
#define LEFT_BACKWARD1 1
#define LEFT_FORWARD2 1
#define LEFT_BACKWARD2 0

#define RIGHT_FORWARD1 0
#define RIGHT_BACKWARD1 1
#define RIGHT_FORWARD2 1
#define RIGHT_BACKWARD2 0

int speedLeft = 0;
int speedRight = 0;

//reflectance sensor parameters
#define LEFT_REF_PIN  9
#define RIGHT_REF_PIN 8

#define SENSOR_HYS 20   //hysteresis in the reflectance sensors

unsigned long leftMin  = 10000;//LEFT_THRESHOLD;
unsigned long leftMax  = 0;//LEFT_THRESHOLD;
unsigned long rightMin = 10000;//RIGHT_THRESHOLD;
unsigned long rightMax = 0;//RIGHT_THRESHOLD;

unsigned long leftThreshold = (leftMin + leftMax) / 2;
unsigned long rightThreshold = (rightMin + rightMax) / 2;

//#define ORDER_COMPLETION_DELAY 15000 //for demoing with an LED
#define SERVO_PIN 7
Servo myServo;

////////////////////////////////////////////////
//order information -- hard-coded for the moment
////////////////////////////////////////////////
uint32_t ip = 0;

uint16_t order_id = 0;
char warehouse = 0;
char delivery_address = 0;

void displayMACAddress(void)
{
  uint8_t macAddress[6];
  
  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}

unsigned long lastOrderCheck = 0;

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Calibrate();
  digitalWrite(13, LOW);
  pinMode(13, INPUT); //just in case

#ifndef __DEBUG__  
  /* Initialise the module */
  Serial.print(F("\nInit..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Failed."));
    while (1) {};
  }

  //displayMACAddress();

  Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY))
  {
    Serial.println(F("Failed!"));
    while (1);
  }

  Serial.println("Trying dhcp");
  /* Wait for DHCP to complete */
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
    Serial.println("failed dhcp");
  }

  /* Display the IP address DNS, Gateway, etc. */
/*  while (!displayConnectionDetails())
  {
    delay(1000);
  }*/

  // Try looking up the website's IP address
  Serial.print(destServer); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(destServer, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
#endif

  pinMode(OUT_R1, OUTPUT);
  pinMode(OUT_R2, OUTPUT);
  pinMode(OUT_L1, OUTPUT);
  pinMode(OUT_L2, OUTPUT);

  pinMode(SPEED_R, OUTPUT);
  pinMode(SPEED_L, OUTPUT);

  attachInterrupt(0, EncoderUpdate, CHANGE);

  delay(2000);
}

void EncoderUpdate(void)
{
  deliveryDistance++;
}

void loop() {
  if (state != WAITING_FOR_ORDER)
  {
    int sensorsReadingBlack = FollowLine();
    if (sensorsReadingBlack == 2) HandleIntersection();
  }

  else if(millis() - lastOrderCheck > 10000) 
  {
    if (CheckForNewOrder())  BeginDelivery();
    lastOrderCheck = millis();
  }  
}

/*
Basic line following behaviour. Checks reflectance sensors and adjusts steering as appropriate.
Returns number of sensors reading black: 0 for neither, 1 for right, 2 for both.
*/

int FollowLine(void)
{
  long leftReading = ReadReflectanceSensor(LEFT_REF_PIN);
  long rightReading = ReadReflectanceSensor(RIGHT_REF_PIN);

//for testing
//    Serial.print(leftReading);
//    Serial.print('\t');
//    Serial.println(rightReading);

  float leftSensor  = (leftReading - leftMin) / (float)(leftMax - leftMin);
  float rightSensor = (rightReading - rightMin) / (float)(rightMax - rightMin);

  Serial.print(leftSensor);
  Serial.print('\t');
  Serial.println(rightSensor);

  int sensorsReadingBlack = (leftSensor > 0.6) + (rightSensor > 0.6); //make sure their reading black
  //Serial.print(sensorsReadingBlack);

  //Control effort: squaring makes it work better
  speedLeft  = BASE_SPEED * (1.0 - leftSensor) * (1.0 - leftSensor);
  speedRight = BASE_SPEED * (1.0 - rightSensor) * (1.0 - rightSensor);

  if (speedLeft < BASE_SPEED / 3) speedLeft = BASE_SPEED / 3;
  if (speedRight < BASE_SPEED / 3) speedRight = BASE_SPEED / 3;

  if (speedLeft > 255) speedLeft = 255;
  if (speedRight > 255) speedRight = 255;

  analogWrite(SPEED_L, speedLeft);
  analogWrite(SPEED_R, speedRight);

//    Serial.print(" ");
//    Serial.print(speedLeft);
//    Serial.print(" ");
//    Serial.println(speedRight);

  //Serial.println(sensorsReadingBlack);
  return sensorsReadingBlack;
}

//////////////////////////////////////////////
//reflectance sensor routines
//////////////////////////////////////////////

unsigned long ReadReflectanceSensor(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(20);
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW); //unneeded...
  unsigned long startUS = micros();
  while (digitalRead(pin) == HIGH) {}
  unsigned long decayTime = micros() - startUS;

  return decayTime;
}

boolean CheckLeftReflectanceSensor(void)
{
  static boolean lastRead = LIGHT;
  unsigned long threshold = leftThreshold + (lastRead == LIGHT ? SENSOR_HYS : -SENSOR_HYS);
  unsigned long reading = ReadReflectanceSensor(LEFT_REF_PIN);
  //  Serial.print(threshold);
  //  Serial.print('\t');
  //  Serial.println(reading);
  lastRead = reading > threshold; //DARK is long
  return lastRead;
}

boolean CheckRightReflectanceSensor(void)
{
  static boolean lastRead = LIGHT;
  unsigned long threshold = rightThreshold + (lastRead == LIGHT ? SENSOR_HYS : -SENSOR_HYS);
  unsigned long reading = ReadReflectanceSensor(RIGHT_REF_PIN);
  //  Serial.print(threshold);
  //  Serial.print('\t');
  //  Serial.println(reading);
  lastRead = reading > threshold; //DARK is 1
  return lastRead;
}

#define HALF_LENGTH 10 //number of ticks needed to move the robot half-way through the intersection
void HandleIntersection(void)
{
  //Serial.println(F("X!"));

  unsigned long startDist = deliveryDistance;
  while (deliveryDistance - startDist < HALF_LENGTH) {
    FollowLine();
    //Serial.println(deliveryDistance - startDist);
  }

  FullStop(); //stop and reset a second...or at least half a second
  delay(500);

  //Serial.println("...");

  switch (state)
  {
    case ROAD_W:
      if (destination == WAREHOUSE_B || destination == WAREHOUSE_C) {
        TurnLeft();
        state = ROAD_BC;
      }
      else if (destination == WAREHOUSE_A) {
        DriveForward();
        state = ROAD_A;
      }
      else {
        DriveForward();  //heading towards houses
        state = ROAD_H;
      }
      break;
    case ROAD_BC:
      if (destination == WAREHOUSE_B) {
        TurnRight();
        state = ROAD_B;
      }
      else if (destination == WAREHOUSE_C) {
        DriveForward();
        state = ROAD_C;
      }
      else {
        TurnRight();
        state = ROAD_W;
      }
      break;
    case ROAD_A:
      if (destination == WAREHOUSE_A) Pickup();
      else {
        DriveForward();
        state = ROAD_W;
      }
      break;
    case ROAD_B:
      if (destination == WAREHOUSE_B) Pickup();
      else {
        TurnLeft();
        state = ROAD_BC;
      }
      break;
    case ROAD_C:
      if (destination == WAREHOUSE_C) Pickup();
      else {
        DriveForward();
        state = ROAD_BC;
      }
      break;
    case ROAD_H:
      if (destination == HOUSE_1) {
        TurnLeft();
        state = ROAD_1;
      }
      else if (destination == HOUSE_2) {
        TurnRight();
        state = ROAD_2;
      }
      else if (destination == HQ) {
        TurnLeft();
        state = ROAD_Q;
      }
      else {
        DriveForward();
        state = ROAD_345;
      }
      break;
    case ROAD_345:
      if (destination == HOUSE_3) {
        TurnLeft();
        state = ROAD_3;
      }
      else if (destination == HOUSE_4) {
        TurnRight();
        state = ROAD_4;
      }
      else if (destination == HQ) {
        DriveForward();
        state = ROAD_H;
      }
      else {
        DriveForward();
        state = ROAD_5;
      }
      break;
    case ROAD_1:
      if (destination == HOUSE_1) DropOff();
      else {
        TurnRight();
        state = ROAD_H;
      }
      break;
    case ROAD_2:
      if (destination == HOUSE_2) DropOff();
      else {
        TurnLeft();
        state = ROAD_H;
      }
      break;
    case ROAD_3:
      if (destination == HOUSE_3) DropOff();
      else {
        TurnRight();
        state = ROAD_345;
      }
      break;
    case ROAD_4:
      if (destination == HOUSE_4) DropOff();
      else {
        TurnLeft();
        state = ROAD_345;
      }
      break;
    case ROAD_5:
      if (destination == HOUSE_5) DropOff();
      else {
        DriveForward();
        state = ROAD_345;
      }
      break;
    case ROAD_Q:
      if (destination == HQ) {
        Spin180();
        state = WAITING_FOR_ORDER;
        WriteCompletedOrder(order_id, deliveryDistance, millis() - deliveryStartTime);
      }
      else {
        TurnLeft();
        state = ROAD_W;
      }
      break;
  }

  delay(100);
  DriveForward();
  //  Serial.print(destination);
  //  Serial.print('\t');
  //  Serial.println(state);
}

///////////////////////////////////
//Basic motion commands
///////////////////////////////////

void FullStop(void)
{
  speedLeft = speedRight = 0;
  analogWrite(SPEED_R, 0);
  analogWrite(SPEED_L, 0);
}

void DriveForward(void)
{
  Serial.print("F: ");
  Serial.print(destination);
  Serial.print('\t');
  Serial.println(state);

  digitalWrite(OUT_R1, RIGHT_FORWARD1);
  digitalWrite(OUT_R2, RIGHT_FORWARD2);

  digitalWrite(OUT_L1, LEFT_FORWARD1);
  digitalWrite(OUT_L2, LEFT_FORWARD2);

  speedLeft = BASE_SPEED;
  speedRight = BASE_SPEED;
}

void TurnRight(void)
{
  Serial.println("Right.");
  analogWrite(SPEED_L, BASE_SPEED * .5);
  digitalWrite(OUT_R1, RIGHT_BACKWARD1);
  digitalWrite(OUT_R2, RIGHT_BACKWARD2);
  analogWrite(SPEED_R, BASE_SPEED * .5);

  while (CheckRightReflectanceSensor() == DARK) {} //in case we started on black
  while (CheckRightReflectanceSensor() == LIGHT) {} //then while white
  while (CheckRightReflectanceSensor() == DARK) {} //then cross the line

  FullStop();
  delay(200);
}

void TurnLeft(void)
{
  Serial.println("L");
  analogWrite(SPEED_R, BASE_SPEED * .5);
  digitalWrite(OUT_L1, LEFT_BACKWARD1);
  digitalWrite(OUT_L2, LEFT_BACKWARD2);
  analogWrite(SPEED_L, BASE_SPEED * .5);

  while (CheckLeftReflectanceSensor() == DARK) {} //in case we started on black
  while (CheckLeftReflectanceSensor() == LIGHT) {} //then while white
  while (CheckLeftReflectanceSensor() == DARK) {} //then cross the line

  //Serial.println("Light.");

  FullStop();
  delay(200);
}

void Spin180(void)
{
  Serial.println("LL");
  TurnLeft();
  TurnLeft();
}

///////////////////////////////////////////
//Package handling
///////////////////////////////////////////

void Pickup(void)
{
  Serial.println(F("Picking."));
  delay(3000); //weak
  Spin180();
  destination = delivery_address;
  DriveForward();
}

void ThrowPackage(void)
{
  Serial.println(F("Delivering."));
  myServo.attach(SERVO_PIN);
  myServo.write(0);
  delay(500);
  myServo.write(180);
  delay(500);
  myServo.detach();
}

void DropOff(void)
{
  ThrowPackage();
  Spin180();
  destination = HQ;
  DriveForward();
}

///////////////////////////////////////////
//Order handling functions
///////////////////////////////////////////
void BeginDelivery(void)
{
  state = ROAD_Q; 
  destination = warehouse;
  deliveryStartTime = millis();
  deliveryDistance = 0;
  DriveForward();
}

String currSerialLine;
unsigned long lastRead = 0;

boolean CheckForNewOrder(void)
{

#ifdef __DEBUG__
  order_id = 0;
  warehouse = WAREHOUSE_A;
  delivery_address = HOUSE_1;

  return true;
#endif

  boolean retVal = false;
  Serial.println(F("Check for new orders."));

  Adafruit_CC3000_Client tlpClient = cc3000.connectTCP(ip, 80);

  if (tlpClient.connected()) 
  {
    Serial.print(F("Check orders..."));
    // make HTTP request to get setpoint:
    tlpClient.fastrprint(F("GET /~gcl8a/bot/checkorders.php HTTP/1.1\r\n"));
    tlpClient.fastrprint(F("Host: "));
    tlpClient.print((destServer));
    tlpClient.fastrprint(F("\r\nConnection:close\r\n"));
    tlpClient.fastrprint(F("\r\n\r\n"));
  }
  else
  {
    Serial.println(F("No connect"));
    return false;
  }

  Serial.println(F("Done"));

  lastRead = millis();
  while(tlpClient.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS))
  {
    // read incoming bytes:
    char inChar = tlpClient.read();
    //Serial.print(inChar);

    // add incoming byte to end of line:
    if (inChar != '\n' && inChar != '\r')
      currSerialLine += inChar;

    // if you get a newline, convert the string to integer setpoint:
    if (inChar == '\n')
    {
      //make a copy of the string to parse and clear the serial string now
      String processString = currSerialLine;
      currSerialLine = "";

      if (processString.startsWith("<order>") && processString.endsWith("</order>"))
      {
        Serial.println(F("New order!"));

        if (state != WAITING_FOR_ORDER)
        {
          Serial.println(F("Busy."));
          break;
        }

        //parse
        order_id = processString.substring(7).toInt(); //this will break at the colon
        Serial.println(order_id);
        int colon = processString.indexOf(":");

        char wh = processString[colon + 1];
        Serial.println(wh);
        if (wh == 'A') warehouse = WAREHOUSE_A;
        else if (wh == 'B') warehouse = WAREHOUSE_B;
        else if (wh == 'C') warehouse = WAREHOUSE_C;
        else break;

        int comma = processString.indexOf(',', colon + 1);

        char address = processString[comma + 1];
        if (address == '1') delivery_address = HOUSE_1;
        else if (address == '2') delivery_address = HOUSE_2;
        else if (address == '3') delivery_address = HOUSE_3;
        else if (address == '4') delivery_address = HOUSE_4;
        else if (address == '5') delivery_address = HOUSE_5;
        else break;

        Serial.println(address);

        Serial.println(F("Done."));
        
        retVal = true;
        lastRead = millis();
      }
    }
  }

  tlpClient.close();

  return retVal;
}


/////////////////////////
//Calibration routine
/////////////////////////
#define CALIBRATION_TIME 5000UL

void Calibrate(void)
{
  FullStop();

  Serial.println(F("Calibrating."));
  delay(1000);

  unsigned long startTime = millis();
  while (millis() - startTime < CALIBRATION_TIME)
  {
    unsigned long leftRef = ReadReflectanceSensor(LEFT_REF_PIN);
    unsigned long rightRef = ReadReflectanceSensor(RIGHT_REF_PIN);

    if (leftRef < leftMin) leftMin = leftRef;
    if (leftRef > leftMax) leftMax = leftRef;

    if (rightRef < rightMin) rightMin = rightRef;
    if (rightRef > rightMax) rightMax = rightRef;

    Serial.print(leftMin);
    Serial.print("  ");
    Serial.print(leftMax);
    Serial.print("  ");
    Serial.print(leftThreshold);
    Serial.print("  ");
    Serial.print(rightMin);
    Serial.print("  ");
    Serial.print(rightMax);
    Serial.print("  ");
    Serial.println(rightThreshold);
  }

  leftThreshold = (2 * leftMin + leftMax) / 3;
  rightThreshold = (2 * rightMin + rightMax) / 3;

  Serial.println(F("Done."));
  delay(2000);
}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if (!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("No IP addr!\r\n"));
    return false;
  }
  else
  {
    Serial.println(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    //Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    //Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    //Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    //Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    //Serial.println();
    return true;
  }
}

void WriteCompletedOrder(int order_id, long distance, long elapsed)
{
  Serial.print("Del: ");
  Serial.println(order_id);

  cc3000.printIPdotsRev(ip);
  Adafruit_CC3000_Client tlpClient = cc3000.connectTCP(ip, 80);
  if (tlpClient.connected())
  {
    //create the sql statement
    //the location of our script
    tlpClient.fastrprint(F("GET /~gcl8a/bot/delivered.php?id="));
    tlpClient.print(order_id);
    tlpClient.print(F("&distance="));
    tlpClient.print(distance);
//    tlpClient.print(F("&elapsed="));
//    tlpClient.print(elapsed);

    tlpClient.fastrprint(F(" HTTP/1.1\r\n"));
    tlpClient.fastrprint(F("Host: "));
    tlpClient.print(destServer);
    tlpClient.fastrprint(F("\r\nConnection:close\r\n"));
    tlpClient.fastrprint(F("\r\n\r\n"));
    
    tlpClient.close();
  }

  else {
    Serial.println(F("Fail!"));
    //return;
  }

  Serial.println(F("Done!"));
}

