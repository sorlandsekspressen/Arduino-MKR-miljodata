// Koden er til vannstand med trykkmåler, HC-SR04, RTC og DS18B20

#include <DS18B20.h> 
#include <OneWire.h> //Trengs?
#include <MKRNB.h>
#include <WDTZero.h>
#include <RTClib.h>
#include <NewPing.h>

WDTZero myWDT; 
NBClient client;
GPRS gprs;
NB nbAccess;
DS18B20 ds(0);
RTC_DS1307 rtc;
NewPing sonar(3, 2, 300); //(Trig,Echo, Max Distance in cm )

//#define printState;      //comment out to turn off all Serial print
#define shutOffPin 1

uint8_t address[] = {40, 250, 31, 218, 4, 0, 0, 52};
uint8_t selected;
char server[] = "sensor.marin.ntnu.no";
char path[] = "/cgi-bin/tof.cgi?";
char filename[]="Vannstand_skien.txt";  
int port = 80; 
bool connected = false; // Status oppkobling
char buffer[128];

int restTid = 4000; //antall ms programmet kjører etter sending
float temperature = 0;
int distance = 0;
float dyb = 0;
float dyb_arr[] = {0, 0, 0, 0, 0};
int dyb_arr_length = sizeof(dyb_arr) / sizeof(dyb_arr[0]);
int dist_arr[] = {0, 0, 0, 0, 0};
int dist_arr_length = sizeof(dist_arr) / sizeof(dist_arr[0]);
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
  rtc.begin();         // start RTC
  delay(1000);         //RTC trenger litt tid på oppstart før lesing

  //when shutdown initialize shutDownFunc()
  myWDT.attachShutdown(shutDownFunc); 
  myWDT.setup(WDT_SOFTCYCLE2M);
  
  readRTC();
  readTemp();
  readDistance();
  readBattVoltage();
  readDyb();
  qsort(dyb_arr, dyb_arr_length, sizeof(dyb_arr[0]), isort); //sorterer dyb_arr
  qsort(dist_arr, dist_arr_length, sizeof(dist_arr[0]), isort); //sorterer dist_arr
  makeString();
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
  DateTime now = rtc.now();
  epochTime = now.unixtime(); 
  }

void readTemp(){
  temperature = ds.getTempC();
  }

void readDistance() {
  
    for (int i = 0; i <= 4; i++) {
      delay(500);
      dist_arr[i] = sonar.ping_cm();
    } 
}

void readBattVoltage(){
  
  battVolt=analogRead(A1)*2*3.3/4096;
  }

void readDyb(){

    for (int i = 0; i <= 4; i++) {
      dyb = 0.24373*analogRead(A0) - 1.6038; //funk fra kalibrering vB->cm
      dyb_arr[i] = dyb;
      delay(1000);
    } 
}


int isort(const void *cmp1, const void *cmp2) {
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  return b - a;
}


void makeString(){
    sprintf(buffer,
            "%s%s,%u,%.1f,%i,%.0f,%.2f",
            path, filename, epochTime, temperature, dist_arr[2], dyb_arr[2], battVolt);
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
