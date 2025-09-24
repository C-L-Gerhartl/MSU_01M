#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// Klemme 15 Fahrzeug
const int pinKlemme15 = 2;
// USB verfügbar?
const int pinComputerStatus = 3;
// Relais Ansteuerung
const int pinRelais = 4;

// MAC-Adresse des Arduino
byte arduinoMac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// IP-Adresse des Arduino
IPAddress arduinoIP(192,168,178,1);

// MAC-Adresse des NUC
byte nucMac[] = { 0x88, 0xae, 0xdd, 0x68, 0xd7, 0x78 };
// IP-Adresse des NUC
IPAddress broadcastIP(192,168,178,10);

unsigned int port = 5000;

EthernetUDP Udp;

void setup() {
  // put your setup code here, to run once:
  Ethernet.begin(arduinoMac, arduinoIP);
  Serial.begin(115200);
  Udp.begin(port);
  pinMode(pinKlemme15, INPUT);
  pinMode(pinComputerStatus, INPUT);
  pinMode(pinRelais, OUTPUT);
  digitalWrite(pinRelais, LOW);
  delay(1000); // Auf Netzwerk warten
  //shutDown();
}

void loop() {
  // put your main code here, to run repeatedly:
  bool klemme15Aktiv = digitalRead(pinKlemme15);
  bool computerLaeuft = digitalRead(pinComputerStatus);
  if(klemme15Aktiv){
    if(!computerLaeuft){
      //digitalWrite(pinRelais, HIGH);  //Start PC per WOL oder WOP
      //delay(500);
      //digitalWrite(pinRelais, LOW);
      sendWOL(nucMac);
      Serial.write("PC startet...\n");
    }
    else{
      // PC läuft, tue nichts
      Serial.write("PC läuft...\n");
    }
  }
  else{
    if(!computerLaeuft){
      // Sleepmode Arduino
      Serial.write("PC aus. Arduino sleep.\n");
    }
    else{
      // PC-Shutdown
      Serial.write("PC wird heruntergefahren.\n");
      shutDown();
    }
  }
  delay(1000);
}

void sendWOL(byte *mac){
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
}

void shutDown(){
  const char* message = "shutdown";
  Udp.beginPacket(broadcastIP, port);
  Udp.write(message);
  Udp.endPacket();
}