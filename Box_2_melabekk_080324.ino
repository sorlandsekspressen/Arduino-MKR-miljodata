#include <DS18B20.h> 
#include <OneWire.h>
#include <Arduino_MKRENV.h>
#include <SD.h>
#include <SPI.h>
#include <MKRNB.h>
#include <WDTZero.h>
#include <RTClib.h>
#include <NewPing.h>

WDTZero myWDT; 
NBClient client;
GPRS gprs;
NB nbAccess;
DS18B20 ds(0);
File myFile;
RTC_DS1307 rtc;
NewPing sonar(3, 2, 200); //(Trig,Echo, Max Distance in cm )

//#define printState;      //comment out to turn off all Serial print
#define shutOffPin 1

uint8_t address[] = {40, 250, 31, 218, 4, 0, 0, 52};
uint8_t selected;
char server[] = "sensor.marin.ntnu.no";
char path[] = "/cgi-bin/tof.cgi?";
char filename[]="BOX_2_maelabekk.txt";  
int port = 80; 
bool connected = false; // Status oppkobling
char buffer[128];

int restTid = 4000; //antall ms programmet kjører etter sending
bool sdState = false;  // For å holde orden på om data ble lagret på SD-kort
float w_temperature = 0;
int dist_arr[] = {0, 0, 0, 0, 0};
int dist_arr_length = sizeof(dist_arr) / sizeof(dist_arr[0]);
float temperature=0;
float humidity=0;
float pressure=0;
float battVolt=0;
unsigned long epochTime;

void setup() {
  pinMode(shutOffPin, OUTPUT);
  digitalWrite(shutOffPin,HIGH);
  #ifdef printState 
    Serial.begin(9600);
    while (!Serial){}     
  #endif
    
  analogReadResolution(12); // 12Bits
  SPI.begin();
  rtc.begin();         // start RTC
  delay(1000);         //RTC trenger litt tid på oppstart før lesing

  //when shutdown initialize shutDownFunc()
  myWDT.attachShutdown(shutDownFunc); 
  myWDT.setup(WDT_SOFTCYCLE2M);
  
  readRTC();
  readWaterTemp();
  readDistance();
  qsort(dist_arr, dist_arr_length, sizeof(dist_arr[0]), isort); //sorterer dist_arr
  readBattVoltage();
  readENVData();
  makeString();
  sdPrint();
  #ifdef printState
    serialPrint();
  #endif
  connectToGPRS();
  connectToServer();  
  delay(1000);
  //Tells Picaxe to turn off power
  digitalWrite(shutOffPin,LOW);  

  }

void readRTC() {
  DateTime now = rtc.now(); //Leser tiden fra RTC
  epochTime = now.unixtime(); // Legger tiden i uTime 
  }

void readWaterTemp(){
  w_temperature = ds.getTempC();
  }

void readDistance() {
  
    for (int i = 0; i <= 4; i++) {
      delay(500);
      dist_arr[i] = sonar.ping_cm();
    } 
}

int isort(const void *cmp1, const void *cmp2) {
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  return b - a;
}

void readBattVoltage(){
  
  battVolt=analogRead(A1)*2*3.3/4096;
  }

void readENVData(){
  ENV.begin();
  temperature = ENV.readTemperature();
  humidity = ENV.readHumidity();
  pressure = ENV.readPressure();
}

void makeString(){
    sprintf(buffer,
            "%s%s,%u,%.1f,%i,%.2f,%.2f,%.1f,%.1f",
            path, filename, epochTime, w_temperature, dist_arr[2], battVolt, temperature, humidity, pressure);
  }


void sdPrint(){
  
  if(SD.begin(4))
    sdState = true;
  
  myFile = SD.open("data.txt", FILE_WRITE);

  if (myFile) {
    myFile.println(buffer);
    myFile.close();  
  }
}

void connectToGPRS(){
  #ifdef printState 
    Serial.println("Started connectToGPRS()");    
  #endif
  
  while (!connected) 
  {
    if ((nbAccess.begin("", true, true) == NB_READY) && (gprs.attachGPRS() == GPRS_READY)) 
    {
      connected = true; 
      #ifdef printState 
        Serial.println("GPRS connected");    
      #endif 
    }   
    else{
      delay(1000);
      #ifdef printState 
        Serial.println("connected still False");    
      #endif
    }
  }
  }
void connectToServer(){
  #ifdef printState 
    Serial.println("in connectToServer()");    
  #endif
  
  if (client.connect(server, port))
  {    
    #ifdef printState 
    Serial.println("Connected to Server");    
    #endif

    client.print("GET "); // Gjør et HTTP request:
    client.print(buffer);
    client.print(',');
    client.print(millis() + restTid);  //runTime skives til server  
    client.print(',');
    client.print(sdState);  // Ble data lagret på SD-kort?
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(server);
    client.println("Connection: close");
    client.println();
    
    #ifdef printState 
    Serial.println("String uploaded to Server");    
    #endif
  } 
  else {

    #ifdef printState 
    Serial.println("No connection to server");    
    #endif
  }
  }

void serialPrint(){
  #ifdef printState 
    Serial.println(buffer);   
  #endif
  }

void shutDownFunc(){
  digitalWrite(shutOffPin,LOW);
  delay(1000);
  }

void loop() {}
