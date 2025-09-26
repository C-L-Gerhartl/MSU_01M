#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Klemme 15 Fahrzeug und wake-up
const int pinKlemme15 = 2;
// USB verfügbar?
const int pinComputerStatus = 3;
// Relais Ansteuerung
const int pinRelais = 4;
// Temp-Sensor 1
const int tempSensor1 = A0;
// Temp-Sensor 2
const int tempSensor2 = A1;
// Temp-Sensor 3
const int tempSensor3 = A2;
// PWM Luefter
const int luefterPWM = 10;

bool klemme15Aktiv = false;
bool computerLaeuft = false;

// MAC-Adresse des Arduino
byte arduinoMac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// IP-Adresse des Arduino
IPAddress arduinoIP(192,168,178,1);

// MAC-Adresse des NUC
byte nucMac[] = { 0x88, 0xae, 0xdd, 0x68, 0xd7, 0x78 };
// IP-Adresse des NUC
IPAddress broadcastIP(192,168,178,10);

unsigned int port = 9;
unsigned int shutDownCouter = 0;

//Zeitsteuerung
unsigned long lastTempRead = 0;
const unsigned long tempInterval = 500;
unsigned long lastStatusCheck = 0;
const unsigned long statusCheckInterval = 1000;

EthernetUDP Udp;

void setup() {
  // put your setup code here, to run once:
  Ethernet.begin(arduinoMac, arduinoIP);
  Serial.begin(115200);
  Udp.begin(port);
  pinMode(pinKlemme15, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinKlemme15), wakeUp, RISING);
  delay(2000);
  pinMode(pinComputerStatus, INPUT);
  pinMode(pinRelais, OUTPUT);
  digitalWrite(pinRelais, LOW);
  pinMode(luefterPWM, OUTPUT);  
  delay(10000); // Auf Netzwerk warten

  Serial.print("Link Status: ");
  Serial.println(Ethernet.linkStatus());
  Serial.print("Arduino IP: ");
  Serial.println(Ethernet.localIP());
  logMessage("Arduino gestartet mit IP: ");
  logMessage(Ethernet.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  klemme15Aktiv = digitalRead(pinKlemme15);
  computerLaeuft = digitalRead(pinComputerStatus);

  // Temperaturmessung und Lüftersteuerung
  if (currentTime - lastTempRead >= tempInterval) {
    lastTempRead = currentTime;

    int t1 = analogRead(tempSensor1);
    int t2 = analogRead(tempSensor2);
    int t3 = analogRead(tempSensor3);

    float voltage1 = t1 * (5.0 / 1023.0);
    float voltage2 = t2 * (5.0 / 1023.0);
    float voltage3 = t3 * (5.0 / 1023.0);

    float tempC1 = (voltage1 - 0.5) * 100.0;
    float tempC2 = (voltage2 - 0.5) * 100.0;
    float tempC3 = (voltage3 - 0.5) * 100.0;

    float avgTemp = (tempC1 + tempC2 + tempC3) / 3.0;

    int pwmValue = map(constrain(avgTemp, 25, 35), 25, 35, 0, 255);
    analogWrite(luefterPWM, pwmValue);

    char jsonMessage[100];
    snprintf(jsonMessage, sizeof(jsonMessage),
      "{\"tempC1\":%.1f,\"tempC2\":%.1f,\"tempC3\":%.1f,\"avgTemp\":%.1f,\"pwm\":%d}",
      tempC1, tempC2, tempC3, avgTemp, pwmValue);
    logMessage(jsonMessage);
  }

  if(currentTime - lastStatusCheck >= statusCheckInterval){
    if(klemme15Aktiv){
      if(!computerLaeuft){
        Serial.println("PC startet...");
        logMessage("PC startet...");
        sendWOL(nucMac);
      }
      else{
        // PC läuft, tue nichts
        Serial.println("PC läuft...");
        logMessage("PC laeuft...");
      }
    }
    else{
      if(!computerLaeuft){
        // Sleepmode Arduino
        Serial.println("PC aus. Arduino sleep."); 
        shutDownCouter = 0;
        delay(100);     
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sleep_mode();
      }
      else{
        // PC-Shutdown
        shutDownCouter++;
        if(shutDownCouter > 5){
          hardReset();
        }
        Serial.println("PC wird heruntergefahren.");
        logMessage("PC wird heruntergefahren.");
        shutDown();
      }
    }
  }
}

void sendWOL(byte *mac){
  Serial.println("PC startet per WOL...");
  logMessage("PC startet per WOL...");
  byte magicPacket[102];

  // Magic Packet beginnt mit 6x 0xFF
  for(int i = 0; i < 6; i++){
    magicPacket[i] = 0xFF;
  }

  // Danach 16x die MAC-Adress
  for(int i = 1; i <= 16; i++){
    for(int j = 0; j  < 6; j++){
      magicPacket[i*6+j] = mac[j];
    }
  } 
  Udp.beginPacket(broadcastIP, port);
  Udp.write(magicPacket, sizeof(magicPacket));
  Udp.endPacket();
  delay(5000);
  if(digitalRead(pinKlemme15) && !digitalRead(pinComputerStatus)){
    Serial.println("PC startet per Relais...");
    startPCphysisch();
  }
}

void shutDown(){
  const char* message = "shutdown";
  Udp.beginPacket(broadcastIP, port);
  Udp.write(message);
  Udp.endPacket();
  delay(5000);
}

void startPCphysisch() {
  digitalWrite(pinRelais, HIGH);  //Start PC per WOL oder WOP
  delay(500);
  digitalWrite(pinRelais, LOW);
}

void wakeUp(){
  sleep_disable();
}

void hardReset(){
  shutDownCouter = 0;
  digitalWrite(pinRelais, HIGH);  // PC-hardreset physisch
  delay(15000);
  digitalWrite(pinRelais, LOW);
}

void logMessage(char* message){
  Udp.beginPacket(broadcastIP, port);
  Udp.write(message);
  Udp.endPacket();
}
