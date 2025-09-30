#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <util/atomic.h>

// Klemme 15 Fahrzeug und wake-up
const int pinKlemme15 = 2;
// USB verfügbar?
const int pinComputerStatus = 5;
// Relais Ansteuerung
const int pinRelais = 4;
// Temp-Sensoren OneWire Bus
OneWire DS18_PIN(7);
// PWM Luefter
const int pinluefterPWM = 9;
// Tacho signal Interrupt
const int pinTacho = 3;
// Tacho Variablen
volatile uint16_t tachoCounter = 0;
float rpm = 0;

bool klemme15Aktiv = false;
bool computerLaeuft = false;

constexpr uint16_t HERTZ {25000};
constexpr uint16_t PRESCALER {1};
constexpr uint8_t CLOCKSET {_BV(CS10)};
constexpr uint16_t ICOUNTER {static_cast<uint16_t>((F_CPU / (2UL * PRESCALER * HERTZ)) - 1)};

// MAC-Adresse des Arduino
byte arduinoMac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// IP-Adresse des Arduino
IPAddress arduinoIP(192,168,178,1);

// MAC-Adresse des NUC
byte nucMac[] = { 0x88, 0xae, 0xdd, 0x68, 0xd7, 0x78 };
// IP-Adresse des NUC
IPAddress broadcastIP(192,168,178,10);

// Wakeup-Port
unsigned int port = 9;
// Shutdown Versuche (max 5)
unsigned int shutDownCouter = 0;

//Zeitsteuerung
unsigned long lastTempRead = 0;
const unsigned long tempInterval = 1000;
unsigned long lastStatusCheck = 0;
const unsigned long statusCheckInterval = 1000;

EthernetUDP Udp;

DallasTemperature tempSensors(&DS18_PIN);

void countTachPulse(){
  tachoCounter++;
}

// Log-Message an NUC per LAN
void logMessage(char* message){
  Udp.beginPacket(broadcastIP, port);
  Udp.write(message);
  Udp.endPacket();
}

// Init PWM signal (25kHz)
void pwmInit() {
  DDRB |= _BV(PB1);   // OCR1A Pin (PB1 / D9)
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ICR1 = ICOUNTER; }
}

// Set dutycycle of the PWM signal
// @param dc   0-100%
void pwmSetDutyCycle(uint8_t dc) {
  uint16_t ocr1a = (dc > 100) ? 100 : dc;
  ocr1a = static_cast<uint16_t>((((ICR1 * 10UL * ocr1a) / 100) + 5) / 10);   // Result must be 16Bit
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { OCR1A = ocr1a; }
}

// start PWM signal
void pwmStart() {
  TCCR1A |= _BV(COM1A1);
  TCCR1B |= _BV(CS10);
}

// stop PWM signal
void pwmStop() {
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1A &= ~(_BV(COM1A1));
  PORTB &= ~(_BV(PB1));   // OCR1A Pin (PB1 / D9) to LOW
}

// Enable external interrupts for INT0 and INT1 Pins (D2, D3)
void enableExternalInterrupts() {
  EICRA = _BV(ISC11) | _BV(ISC10) | _BV(ISC01) | _BV(ISC00);   // Int0 & Int1 -> rising edge
  EIMSK = _BV(INT1) | _BV(INT0);
}

// ShutDown
void shutDown(){
  Serial.println("Fährt runter...");
  const char* message = "shutdown";
  Udp.beginPacket(broadcastIP, port);
  Udp.write(message);
  Udp.endPacket();
  Serial.println(message);
  delay(5000);
}

// NUC-Start physisch
void startPCphysisch() {
  digitalWrite(pinRelais, HIGH);  //Start PC per WOL oder WOP
  delay(500);
  digitalWrite(pinRelais, LOW);
}

// Wake up Arduino Interupt
void wakeUp(){
  sleep_disable();
}

// NUC Shut off physisch
void hardReset(){
  shutDownCouter = 0;
  digitalWrite(pinRelais, HIGH);  // PC-hardreset physisch
  delay(15000);
  digitalWrite(pinRelais, LOW);
}

void setup() {
  // put your setup code here, to run once:
  pwmInit();
  pwmSetDutyCycle(100);
  pwmStart();
  Ethernet.begin(arduinoMac, arduinoIP);
  Serial.begin(115200);
  Udp.begin(port);
  tempSensors.begin();
  pinMode(pinKlemme15, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinKlemme15), wakeUp, RISING);
  pinMode(pinTacho, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinTacho), countTachPulse, FALLING);
  delay(2000);
  pinMode(pinComputerStatus, INPUT);
  pinMode(pinRelais, OUTPUT);
  digitalWrite(pinRelais, LOW);
  pinMode(pinluefterPWM, OUTPUT);  
  delay(10000); // Auf Netzwerk warten


  Serial.print("Link Status: ");
  Serial.println(Ethernet.linkStatus());
  Serial.print("Arduino IP: ");
  Serial.println(Ethernet.localIP());
  logMessage("Arduino gestartet mit IP: ");
  logMessage(Ethernet.localIP());
}

// Wake on LAN 
void sendWOL(byte *mac){
  Serial.println("PC startet per WOL...");
  logMessage("PC startet per WOL...");
  byte magicPacket[102];

  // Magic Packet beginnt mit 6x 0xFF
  for(int i = 0; i < 6; i++){
    magicPacket[i] = 0xFF;
    Serial.print(magicPacket[i]);
    Serial.print(",");
  }
  Serial.println("");
  // Danach 16x die MAC-Adress
  for(int i = 1; i <= 16; i++){
    for(int j = 0; j  < 6; j++){
      magicPacket[i*6+j] = mac[j];
      Serial.print(magicPacket[i*6+j]);
      Serial.print(",");
    }
  } 
  Serial.println("");
  Udp.beginPacket(broadcastIP, port);
  Udp.write(magicPacket, sizeof(magicPacket));
  Udp.endPacket();
  delay(5000);
  if(digitalRead(pinKlemme15) && !digitalRead(pinComputerStatus)){
    Serial.println("PC startet per Relais...");
    startPCphysisch();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  klemme15Aktiv = digitalRead(pinKlemme15);
  computerLaeuft = digitalRead(pinComputerStatus);

  // Temperaturmessung und Lüftersteuerung
  if (currentTime - lastTempRead >= tempInterval) {
    lastTempRead = currentTime;

    tempSensors.requestTemperatures(); // Alle Sensoren abfragen
    float temps[5];
    char tempsStr[5][10];
    float tempKumuliert = 0;
    int sensorCount = min(tempSensors.getDeviceCount(), 3);
    for (int i = 0; i < sensorCount; i++) {
      temps[i] = tempSensors.getTempCByIndex(i);
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(temps[i]);
      Serial.println(" °C");
      dtostrf(temps[i], 4, 1, tempsStr[i]);
      tempKumuliert = tempKumuliert + temps[i];
    }
    char avrTempStr[10];
    float avrTemp = tempKumuliert / sensorCount;
    dtostrf(avrTemp, 4, 1, avrTempStr);
    Serial.println(tempKumuliert);

    int pwmValue = map(constrain(avrTemp, 25, 35), 25, 35, 0, 100);
    pwmSetDutyCycle(pwmValue);
    Serial.println(pwmValue);

    // Lüfterdrehzahl ablesen
    noInterrupts();
    uint16_t pulses = tachoCounter;
    tachoCounter = 0;
    interrupts();

    rpm = (pulses / 2.0) * 60.0;
    Serial.print("Lüfterdrehzahl: ");
    Serial.print(rpm);
    Serial.println(" RPM");
    char fanSpeedStr[10];
    dtostrf(rpm, 4, 1, fanSpeedStr);

    char jsonMessage[200];
    snprintf(jsonMessage, sizeof(jsonMessage),
      "{\"tempC1\":%s,\"tempC2\":%s,\"tempC3\":%s,\"avgTemp\":%s,\"pwm\":%d,\"fanSpeed\":%s}",
      tempsStr[0], tempsStr[1], tempsStr[2], avrTempStr, pwmValue, fanSpeedStr);
    logMessage(jsonMessage);
    Serial.println(jsonMessage);
  }

  // NUC Status-Check
  if(currentTime - lastStatusCheck >= statusCheckInterval){
    lastStatusCheck = currentTime;
    if(klemme15Aktiv){
      // PC start
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
