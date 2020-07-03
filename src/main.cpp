#include <SPI.h>
#include <SPIFFS.h>
#include <Arduino.h>
#include <WiFimulti.h>
#include <WebServer.h>
#include <EEPROM.h>

//CS D8  HMOSI D7  HMISO D6  HSCLK D5 for esp8266
//CS GPIO15  HMOSI GPIO13  HMISO GPIO12  HSCLK GPIO14 for esp32
//CS 10 Din 11 Dout 12 CLK 13 mcp3008
#define CSpin 15
#define button 23
#define R 18
#define G 19
#define B 21
#define DefaultSamples 10000
#define buff 40000
#define EEPROM_SIZE 40  //40 bytes

WiFiMulti wifiMulti;
WebServer server(80);
IPAddress local_IP(192, 168, 43, 43);
IPAddress gateway(192, 168, 43, 1);
IPAddress subnet(255, 255, 0, 0);


int buf[2];
int buf2[2];
bool startsampling;
unsigned int stime;
static unsigned short *Value2;
unsigned short Value[buff];
unsigned int Settime = DefaultSamples,prevSettime = DefaultSamples;
unsigned int test = 0;
hw_timer_t *timer123 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
const char* ssid = "HehexD";
const char* password = "00000008";
const char* ssid2 ="ESP32-default";
String ssid3;
String password3;

void IRAM_ATTR onTimer();
void SPITR();
void writetoserial();
void readsignal();
void wifibegin();
void settimerinterrupt();
void handleForm();
void writetoEEPROM(String s,short offset);
void stratEEPROM();

void setup()
{
  pinMode(CSpin, OUTPUT);
  pinMode(button, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  

  SPI.begin(14, 12, 13, 15);   // begin HSPI 
    
  Serial.begin(1152000);  // begin serial
  Serial.println('\n');

  stratEEPROM(); //start EEPROM

  wifibegin();  // begin WIFI
  
  settimerinterrupt();  // begin timer interrupt

  //allocate memory
  // stime = (unsigned int *)heap_caps_malloc(buff, MALLOC_CAP_32BIT);
  // if (stime == NULL)
  // {
  //   Serial.println("ERROR assigning time array!");
  // }
  // else
  // {
  //   Serial.println("Seccessful assigning time array!");
  // }
  Value2 = (unsigned short *)heap_caps_malloc(buff, MALLOC_CAP_8BIT); 
  if (Value2 == NULL)
  {
    Serial.println("ERROR assigning Value 2 array!");
  }
  else
  {
    Serial.println("Seccessful assigning Value 2 array!");
  }
  Serial.println(esp_get_free_heap_size());

  //start SPIFFS
  if (!SPIFFS.begin())
  { /* begin SPIFFS */
    Serial.println("SPIFFS begins failed.");
  }
  else
  {
    Serial.println("SPIFFS begins.");
  }

  //Server reply settings
  server.on("/myscript.js",[](){
    File file = SPIFFS.open("/myscript.js", "r");                 // Open it
    server.streamFile(file, "application/javascript"); // And send it to the client
    file.close();
  });
  server.on("/",[](){
    File file = SPIFFS.open("/index.html", "r");                 // Open it
    server.streamFile(file, "text/html"); // And send it to the client
    file.close();
    delay(100);
  });
  server.on("/data.txt",[](){
    File file = SPIFFS.open("/data.txt", "r");                 // Open it
    server.streamFile(file, "text/plain"); // And send it to the client
    file.close();
  });
  server.on("/set10",[](){
    Settime = 10000;
    timerAlarmWrite(timer123, 1000, true);
    server.send(200, "text/plain", "MCU response: Set sampling time 10s.");
  });
  server.on("/set20",[](){
    Settime = 20000;
    timerAlarmWrite(timer123, 1000, true);
    server.send(200, "text/plain", "MCU response: Set sampling time 20s.");
  });
  server.on("/set30",[](){
    Settime = 30000;
    timerAlarmWrite(timer123, 1000, true);
    server.send(200, "text/plain", "MCU response: Set sampling time 30s.");
  });
  server.on("/set102K",[](){
    Settime = 20000;
    timerAlarmWrite(timer123, 500, true);
    server.send(200, "text/plain", "MCU response: Set sampling time 10s.");
  });
  server.on("/set202K",[](){
    Settime = 40000;
    timerAlarmWrite(timer123, 500, true);
    server.send(200, "text/plain", "MCU response: Set sampling time 20s.");
  });
  server.on("/action_page",[](){
    handleForm();
  });
  server.on("/startsampling",[](){
    startsampling = true;
    server.send(200, "text/plain", "Start sampling.");
  });
  server.begin();                           // Actually start the server
  Serial.println("HTTP server started");
}

void loop(void)
{
  digitalWrite(G, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  if(prevSettime != Settime) {
    Serial.print(Settime);
    Serial.println(" samples let's go!");
    prevSettime = Settime ;
  }
  if(digitalRead(button)==HIGH){
    startsampling = true;
  }
  if (startsampling == true){
    digitalWrite(G, LOW);
    readsignal();
    startsampling = false;
  }
  server.handleClient();
}

void handleForm() {
 String ssidarg = server.arg("ssid"); 
 String PWarg = server.arg("password"); 
 server.sendHeader("Location", "/");
 server.send(302, "text/plain", "Updated– Press Back Button");

 Serial.print("New ssid:");
 Serial.println(ssidarg);
 writetoEEPROM(ssidarg,0);
 Serial.print("New password:");
 Serial.println(PWarg);
 writetoEEPROM(PWarg,EEPROM_SIZE/2);
 EEPROM.commit();
 
}



void settimerinterrupt(){
  timer123 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer123, &onTimer, true);
  timerAlarmWrite(timer123, 1000, true);
  timerAlarmEnable(timer123);
}

void wifibegin(){
  volatile unsigned short  wifidelay = 0;
  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP(ssid2);
  wifiMulti.addAP(ssid3.c_str(), password3.c_str());
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  Serial.println("Connecting ...");
  while (wifiMulti.run() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    digitalWrite(R, !digitalRead(R));
    Serial.print('.');
    wifidelay+=1;
    if(wifidelay > 4){
      Serial.println("Unable to connect Wi-Fi");
      break;
    }
  }
  digitalWrite(R,LOW);
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());              // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());           // Send the IP address of the ESP8266 to the computer
}

void SPITR()
{

  // enable Slave Select
  digitalWrite(CSpin, LOW);
  SPI.transfer(1); // initiate transmission
  for (int pos = 0; pos < 2; pos++)
  {
    buf[pos] = SPI.transfer(144); //128 for single mode, CH0
  }
  // disable Slave Select
  digitalWrite(CSpin, HIGH);

  digitalWrite(CSpin, LOW);
  SPI.transfer(1); // initiate transmission
  for (int pos = 0; pos < 2; pos++)
  {
    buf2[pos]= SPI.transfer(128); //144 for single mode, CH1
  }
  // disable Slave Select
  digitalWrite(CSpin, HIGH);
    
}

void readsignal(void)
{
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(R, HIGH);
  Serial.println("Reading signal from ADC.");
  for (int i = 0; i < Settime; i++)
  {
    if (interruptCounter > 0)
    {
      portENTER_CRITICAL(&timerMux);
      interruptCounter = 0;
      portEXIT_CRITICAL(&timerMux);
      if(micros()-stime >600) Serial.println("");
      stime = micros();
      SPITR();
      buf[0] = (buf[0] & B00000011) << 8;
      buf2[0] = (buf2[0] & B00000011) << 8;
      Value[i] = (buf[0] + buf[1]);
      Value2[i]= (buf2[0] + buf2[1]);
    }
    else
    {
      i--;
    }
  }
  digitalWrite(R, LOW);
  //writing to data.txt
  digitalWrite(B, HIGH);
  File f = SPIFFS.open("/data.txt", "w");
  if (!f) {
    Serial.println("file open failed");
  }
  Serial.println("writing to data.txt");
  f.print ("Time(ms),Channel 1,Channel 2\n");

  for (int i = 0; i < Settime; i++)
  {
    // test = stime[i]-stime[0];
    f.print(i);
    f.print(",");
    f.print((float)(Value[i]) / 1024 * 3.3);
    f.print(",");
    f.print((float)(Value2[i]) / 1024 * 3.3);
    f.print("\n");
  }
  f.close();
  Serial.println("End of printing.");

  Serial.print("sampled ");
  Serial.print(Settime);
  Serial.print("times, ");
  // Serial.print("duration = ");
  // Serial.print(float(stime[Settime-1]-stime[0])/1000);
  // Serial.println("ms");
  
  digitalWrite(B, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void writetoEEPROM(String s,short offset){
  for(int i = 0+offset; i <= offset + s.length(); i++){
    EEPROM.write(i,byte(s.charAt(i-offset)));
  }
}

void stratEEPROM(){
  char r;
  ssid3 = ""; password3 = "";
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
  }
  Serial.println(" bytes read from Flash . Values are:");
  for (int i = 0; i < EEPROM_SIZE/2; i++)
  {
    r = EEPROM.read(i);
    Serial.print(byte(r)); Serial.print(" ");
    if(byte(r)==0){
      break;
    }
    ssid3 += r;
  }
  for (int i = 0+EEPROM_SIZE/2; i < EEPROM_SIZE; i++)
  {
    r = EEPROM.read(i);
    Serial.print(byte(r)); Serial.print(" ");
    if(byte(r)==0){
      break;
    }
    password3 += r;
  }
  Serial.println("");
  Serial.print(ssid3);Serial.print(" ; ");Serial.println(password3);

}
  