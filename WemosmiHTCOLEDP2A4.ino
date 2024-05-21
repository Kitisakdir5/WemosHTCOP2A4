#include <ESP8266WiFi.h>
////////////// Library NETPIE 2020  ///////////////////
#include <PubSubClient.h>
#include <Wire.h>

 #include <SoftwareSerial.h>

///////////////////// OLED //////////////////////
// library OLEDtest2 
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
 ///////////// show logo adafruit  ///////////////  
   Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

///////////// ssid password NETPIE2020  //////////////
const char* ssid = "******" ;
const char* password = "*******" ;

////////////////////  variable int to string ////////////
char angleHstr[7] ;
//////////////////// variable push red green //////////
int pushred = 0 , pushgreen = 0 ,  allanglerg1 = 0 , pushgreen5 = 0, pushred5 = 0 ;
int num1 = 0 ;
/////////////////////// part of NETPIE 2020   ////////////////

////////// Client Token Secret for  Device GY521node1 ////////////
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883 ;
const char* mqtt_Client = "*******";
const char* mqtt_username = "*******";
const char* mqtt_password = "********";
char msg[100];  ///////  can increase msg[120]
long lastMsg = 0;

////////////////////// variable GY-521 ///////////////////////
/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2] , Gyro_angle[2] , Total_angle[2]  ;
float elapsedTime = 0 , time1= 0  ,  timePrev = 0  ;
int i;
float rad_to_deg = 180/3.141592654 ;

/////////////////////////////// variable HWC5883L ////////////////
// Magnatic Declination in Ranong is -0.0116355289151098
#define Declination    -0.01163 // -0.00669
#define hmc5883l_address  0x1E
double angleHeading , angleZ ;
int angleint1 = 0, angleint2 = 0, angleint3 = 0  ;
/////////////////////// END variable HWC5883L ///////////////////

/////////////////// Function HMC5883L /////////////////////////
void hmc5883l_init(){   /* Magneto initialize function */
  Wire.beginTransmission(hmc5883l_address);
  Wire.write(0x00);
  Wire.write(0x70); //8 samples per measurement, 15Hz data output rate, Normal measurement 
  Wire.write(0xA0); //
  Wire.write(0x00); //Continuous measurement mode
  Wire.endTransmission();
  delay(500);
}
int hmc5883l_GetHeading(){
  int16_t x, y, z;
  double Heading;
  Wire.beginTransmission(hmc5883l_address);
  Wire.write(0x03);
  Wire.endTransmission();
  /* Read 16 bit x,y,z value (2's complement form) */
  Wire.requestFrom(hmc5883l_address, 6);
  x = (((int16_t)Wire.read()<<8) | (int16_t)Wire.read());
  z = (((int16_t)Wire.read()<<8) | (int16_t)Wire.read());
  y = (((int16_t)Wire.read()<<8) | (int16_t)Wire.read());

  Heading = atan2((double)y, (double)x) + Declination;
  if (Heading>2*PI) /* Due to declination check for >360 degree */
   Heading = Heading - 2*PI;
  if (Heading<0)    /* Check for sign */
   Heading = Heading + 2*PI;
  return (Heading* 180 / PI);/* Convert into angle and return */
}

WiFiClient espClient;
PubSubClient client(espClient);

///////////// Void of NETPIE2020  ////////////////////
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting NETPIE2020 connection…");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("NETPIE2020 connected");
     client.subscribe("@msg/HMC5883");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

// function callback()

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println(message);
      
  ////////// End void callback  /////////////////
}

//////////////////// Multiplexer I2C //////////////////////////////
// Select I2C BUS
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}



void setup() {

  Serial.begin(115200);  // 250000
/////////////  Push RED GREEN ///////////////
  pinMode(D6, INPUT) ; // green
  pinMode(D5, INPUT) ; // red

 //////////////////  Setup GY-521 0x70 SCL,SDA 2 ///////////////////
  Wire.begin();
  TCA9548A(0) ;
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

///////////////////////  END OLED SETUP ////////////////////////////////

  Wire.begin();
  TCA9548A(1) ;
//  Wire.begin(A4, A5); /* join i2c bus with SDA=D6 and SCL=D5 of NodeMCU */
  hmc5883l_init();




/////////////////////// OLED SETUP ////////////////////////////////
  Wire.begin() ;
  TCA9548A(2) ;
  display.begin(i2c_Address, true); // Address 0x3C default
 //display.setContrast (0); // dim display
 
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();

   TCA9548A(2) ;
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("TEST");
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.println("Compass");
  display.display();
  delay(2000);
  display.clearDisplay();  

//////////////////////  SETUP LOOP For NETPIE 2020  //////////////////
  Serial.println("Starting...");
  if (WiFi.begin(ssid, password)) {
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); 
  time1 = millis(); //Start counting time in milliseconds

}//end of setup void

void loop() {
/////////////////////////// GY-521 in Void loop //////////////////////////////////////
     timePrev = time1 ;  // the previous time is stored before the actual time read
     time1 = millis() ;  // actual time read
     elapsedTime = (time1 - timePrev) / 1000; 
 
     TCA9548A(0) ;
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 

    /// RAW data Acc  
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
   TCA9548A(0) ;
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   /// RAW Data gyro  
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

  //   Serial.print("gyroangle  ");
  //   Serial.print(Gyro_angle[0]) ; Serial.print("\t") ;
  //   Serial.print(Gyro_angle[1]) ; Serial.print("\t") ;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
    
   //  Serial.print("Total angle ");
  //   Serial.print(Total_angle[1]) ;    Serial.print("\t") ; // X axis
  //   Serial.print(Total_angle[0]);
  //   Serial.print("\n") ; 
  
/////////////// check push //////////////////////////
  pushgreen = digitalRead(D6) ;
  pushred = digitalRead(D5) ;   
////////// find angle ///////// 
    TCA9548A(1) ;
    angleHeading = hmc5883l_GetHeading();
    angleint1 = int(angleHeading) ;
    angleint2 = angleint1 ;
/*
if ( pushgreen == 1 ) {
    angleint2 = angleint1 ;
}
if ( pushgreen == 0 ) {
    angleint2 = angleint3 ;
}        
*/
   //////// pushred1 is shoot ///////////
  pushred5 = 10000 * pushred ; 
 //////// pushgreen1 is slide ///////////
  pushgreen5 = 1000 * pushgreen ; 
///////// add all angle + pushred5 + pushgreen5 ////////////
 allanglerg1 = pushred5 + pushgreen5 + angleint2 ; // angleint2
 angleint3 = angleint2 ;

////////////// Cal angle HMC5883L //////////////
 
   ////////////// Void Loop HMC5883L //////////////////
    Serial.print("number ");
    Serial.print(num1);
    Serial.print(" - ");
    Serial.print("Azimuth Angle : ");
    Serial.print(angleint2); //
    Serial.print(" - ");
    Serial.print("angle[1]: ");
    Serial.print(Total_angle[1]);
    Serial.print(" - ");
    Serial.print("angle[0]: ");
    Serial.print(Total_angle[0]);
    Serial.print(" - ");
    Serial.print("Pushgreen : ");
    Serial.print(pushgreen);
    Serial.print(" - ");
    Serial.print("Pushred: ");
    Serial.print(pushred);
    Serial.print("\n");

 //////////////// NETPIE2020 in Void LOOP /////////////////
     if (!client.connected()) {
    reconnect();
  }
  client.loop();


////////////////////// OLED /////////////////////////
/////     display.setCursor(0,0); , (width , height) ,(128, 64)
      TCA9548A(2) ;
       // display label angle
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Angle[2]:");

      // display angle Z
      display.setTextSize(1);
      display.setCursor(80,0); /// (0,0) 
      display.print(angleint2);  //
   
       // display label push red
      display.setTextSize(1);
      display.setCursor(0,12); // (0,25) 
      display.print("Push RED :");

      // display pushred
      display.setTextSize(1);
      display.setCursor(80,12); /// (0,25) 
      display.print(pushred);
    
       // display label push green
      display.setTextSize(1);
      display.setCursor(0,24); // (0,50)
      display.print("Push GREEN :");

      // display pushgreen
      display.setTextSize(1);
      display.setCursor(80,24); /// (0,50) 
      display.print(pushgreen);
      display.display(); 

       // display label Total angle [1]
      display.setTextSize(1);
      display.setCursor(0,36); // (0,50)
      display.print("angle[1]:"); /// angle[1]:

      // display Total angle [1]
      display.setTextSize(1);
      display.setCursor(80,36); /// (0,50) 
      display.print(angleint1);
      display.display(); 


       // display label Total angle [0]
      display.setTextSize(1);
      display.setCursor(0,48); // (0,50)
      display.print("SEND:");

      // display Total angle [0]
      display.setTextSize(1);
      display.setCursor(80,48); /// (0,50) 
      display.print(allanglerg1); 
      display.display(); 


      delay(200);
      display.clearDisplay();

  ////////////// double to  string /////////////
  dtostrf(allanglerg1 ,7,0,angleHstr);  // 8,6

  client.publish("@msg/HMC5883", angleHstr );
  Serial.println("Hello NETPIE2020");

  Serial.print("allanglerg1 = ");
  Serial.print(allanglerg1);
  Serial.print("\n");


  delay(200) ;


}//end of loop void



