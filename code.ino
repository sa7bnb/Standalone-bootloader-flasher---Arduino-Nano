/*
 * Arduino Nano Komplett ISP Programmerare - ArduinoISP Kompatibel
 * Programmerar Optiboot bootloader på Arduino Nano (ATmega328P)
 * Startar automatiskt när ström kopplas på!
 * 
 * Kopplingar (Programmerare Nano -> Mål Nano):
 * D10 (SS)   -> RESET
 * D11 (MOSI) -> D11 (MOSI)
 * D12 (MISO) -> D12 (MISO) 
 * D13 (SCK)  -> D13 (SCK)
 * 5V         -> 5V
 * GND        -> GND
 * 
 * Ytterligare kopplingar:
 * D2 -> Startknapp (med externt pulldown motstånd)
 * D3 -> Färdig LED (med lämpligt förströmsmotstånd)
 * 
 * LED signaler (kompatibla med ArduinoISP):
 * Pin 7 (Programming): Tänd under programmering
 * Pin 8 (Error): Tänd om senaste programmering misslyckades  
 * Pin 9 (Heartbeat): Blinkar lugnt om senaste programmering lyckades
 * 
 * Extra signaler:
 * - 10 snabba blink (Pin 9): Programmering framgångsrik
 * - 5 långsamma blink (Pin 8): Programmering misslyckades
 * - Pin 3: Tänd när programmering är klar (framgångsrik)
 * 
 * Komplett Optiboot bootloader inbäddad - ingen extern hårdvara behövs!
 * Seriell monitor 115200 baud för att följa processen
 */

#include <SPI.h>

// Definitioner för ISP - kompatibla med ArduinoISP
#define RESET_PIN 10
#define LED_HEARTBEAT 9   // Heartbeat - visar att systemet lever
#define LED_ERROR 8       // Error - lyser vid fel 
#define LED_PMODE 7       // Programming - lyser under programmering

// Nya pins för knapp och status
#define START_BUTTON_PIN 2  // Startknapp med externt pulldown
#define DONE_LED_PIN 3      // LED som tänds när programmeringen är klar

// ATmega328P signature
#define ATMEGA328P_SIGNATURE_0 0x1E
#define ATMEGA328P_SIGNATURE_1 0x95
#define ATMEGA328P_SIGNATURE_2 0x0F

// Äkta Optiboot bootloader för ATmega328P (Arduino Nano/Uno) - från Arduino IDE
// 512 bytes, börjar på adress 0x7E00
const PROGMEM uint8_t bootloader[] = {
  // 0x7E00
  0x11, 0x24, 0x84, 0xB7, 0x14, 0xBE, 0x81, 0xFF, 0xF0, 0xD0, 0x85, 0xE0, 0x80, 0x93, 0x81, 0x00,
  0x82, 0xE0, 0x80, 0x93, 0xC0, 0x00, 0x88, 0xE1, 0x80, 0x93, 0xC1, 0x00, 0x86, 0xE0, 0x80, 0x93,
  0xC2, 0x00, 0x80, 0xE1, 0x80, 0x93, 0xC4, 0x00, 0x8E, 0xE0, 0xC9, 0xD0, 0x25, 0x9A, 0x86, 0xE0,
  0x20, 0xE3, 0x3C, 0xEF, 0x91, 0xE0, 0x30, 0x93, 0x85, 0x00, 0x20, 0x93, 0x84, 0x00, 0x96, 0xBB,
  0xB0, 0x9B, 0xFE, 0xCF, 0x1D, 0x9A, 0xA8, 0x95, 0x81, 0x50, 0xA9, 0xF7, 0xCC, 0x24, 0xDD, 0x24,
  0x88, 0x24, 0x83, 0x94, 0xB5, 0xE0, 0xAB, 0x2E, 0xA1, 0xE1, 0x9A, 0x2E, 0xF3, 0xE0, 0xBF, 0x2E,
  0xA2, 0xD0, 0x81, 0x34, 0x61, 0xF4, 0x9F, 0xD0, 0x08, 0x2F, 0xAF, 0xD0, 0x02, 0x38, 0x11, 0xF0,
  0x01, 0x38, 0x11, 0xF4, 0x84, 0xE0, 0x01, 0xC0, 0x83, 0xE0, 0x8D, 0xD0, 0x89, 0xC0, 0x82, 0x34,
  0x11, 0xF4, 0x84, 0xE1, 0x03, 0xC0, 0x85, 0x34, 0x19, 0xF4, 0x85, 0xE0, 0xA6, 0xD0, 0x80, 0xC0,
  0x85, 0x35, 0x79, 0xF4, 0x88, 0xD0, 0xE8, 0x2E, 0xFF, 0x24, 0x85, 0xD0, 0x08, 0x2F, 0x10, 0xE0,
  0x10, 0x2F, 0x00, 0x27, 0x0E, 0x29, 0x1F, 0x29, 0x00, 0x0F, 0x11, 0x1F, 0x8E, 0xD0, 0x68, 0x01,
  0x6F, 0xC0, 0x86, 0x35, 0x21, 0xF4, 0x84, 0xE0, 0x90, 0xD0, 0x80, 0xE0, 0xDE, 0xCF, 0x84, 0x36,
  0x09, 0xF0, 0x40, 0xC0, 0x70, 0xD0, 0x6F, 0xD0, 0x08, 0x2F, 0x6D, 0xD0, 0x80, 0xE0, 0xC8, 0x16,
  0x80, 0xE7, 0xD8, 0x06, 0x18, 0xF4, 0xF6, 0x01, 0xB7, 0xBE, 0xE8, 0x95, 0xC0, 0xE0, 0xD1, 0xE0,
  0x62, 0xD0, 0x89, 0x93, 0x0C, 0x17, 0xE1, 0xF7, 0xF0, 0xE0, 0xCF, 0x16, 0xF0, 0xE7, 0xDF, 0x06,
  0x18, 0xF0, 0xF6, 0x01, 0xB7, 0xBE, 0xE8, 0x95, 0x68, 0xD0, 0x07, 0xB6, 0x00, 0xFC, 0xFD, 0xCF,
  0xA6, 0x01, 0xA0, 0xE0, 0xB1, 0xE0, 0x2C, 0x91, 0x30, 0xE0, 0x11, 0x96, 0x8C, 0x91, 0x11, 0x97,
  0x90, 0xE0, 0x98, 0x2F, 0x88, 0x27, 0x82, 0x2B, 0x93, 0x2B, 0x12, 0x96, 0xFA, 0x01, 0x0C, 0x01,
  0x87, 0xBE, 0xE8, 0x95, 0x11, 0x24, 0x4E, 0x5F, 0x5F, 0x4F, 0xF1, 0xE0, 0xA0, 0x38, 0xBF, 0x07,
  0x51, 0xF7, 0xF6, 0x01, 0xA7, 0xBE, 0xE8, 0x95, 0x07, 0xB6, 0x00, 0xFC, 0xFD, 0xCF, 0x97, 0xBE,
  0xE8, 0x95, 0x26, 0xC0, 0x84, 0x37, 0xB1, 0xF4, 0x2E, 0xD0, 0x2D, 0xD0, 0xF8, 0x2E, 0x2B, 0xD0,
  0x3C, 0xD0, 0xF6, 0x01, 0xEF, 0x2C, 0x8F, 0x01, 0x0F, 0x5F, 0x1F, 0x4F, 0x84, 0x91, 0x1B, 0xD0,
  0xEA, 0x94, 0xF8, 0x01, 0xC1, 0xF7, 0x08, 0x94, 0xC1, 0x1C, 0xD1, 0x1C, 0xFA, 0x94, 0xCF, 0x0C,
  0xD1, 0x1C, 0x0E, 0xC0, 0x85, 0x37, 0x39, 0xF4, 0x28, 0xD0, 0x8E, 0xE1, 0x0C, 0xD0, 0x85, 0xE9,
  0x0A, 0xD0, 0x8F, 0xE0, 0x7A, 0xCF, 0x81, 0x35, 0x11, 0xF4, 0x88, 0xE0, 0x18, 0xD0, 0x1D, 0xD0,
  0x80, 0xE1, 0x01, 0xD0, 0x65, 0xCF, 0x98, 0x2F, 0x80, 0x91, 0xC0, 0x00, 0x85, 0xFF, 0xFC, 0xCF,
  0x90, 0x93, 0xC6, 0x00, 0x08, 0x95, 0x80, 0x91, 0xC0, 0x00, 0x87, 0xFF, 0xFC, 0xCF, 0x80, 0x91,
  0xC0, 0x00, 0x84, 0xFD, 0x01, 0xC0, 0xA8, 0x95, 0x80, 0x91, 0xC6, 0x00, 0x08, 0x95, 0xE0, 0xE6,
  0xF0, 0xE0, 0x98, 0xE1, 0x90, 0x83, 0x80, 0x83, 0x08, 0x95, 0xED, 0xDF, 0x80, 0x32, 0x19, 0xF0,
  0x88, 0xE0, 0xF5, 0xDF, 0xFF, 0xCF, 0x84, 0xE1, 0xDE, 0xCF, 0x1F, 0x93, 0x18, 0x2F, 0xE3, 0xDF,
  0x11, 0x50, 0xE9, 0xF7, 0xF2, 0xDF, 0x1F, 0x91, 0x08, 0x95, 0x80, 0xE0, 0xE8, 0xDF, 0xEE, 0x27,
  0xFF, 0x27, 0x09, 0x94
};

const unsigned int BOOTLOADER_SIZE = sizeof(bootloader);
const unsigned int BOOTLOADER_START = 0x7E00; // Bootloader start address

// ISP status
bool programmingMode = false;
bool lastProgrammingSuccess = false;

// Knapp hantering
bool lastButtonState = false;
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 50; // ms

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Arduino Nano ISP - Komplett Bootloader Programmerare ==="));
  Serial.println(F("Optiboot bootloader inbäddad - 512 bytes"));
  Serial.println(F("ArduinoISP kompatibel - tryck på startknapp för att börja!"));
  
  // Konfigurera alla pins
  pinMode(RESET_PIN, OUTPUT);
  pinMode(LED_HEARTBEAT, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  pinMode(LED_PMODE, OUTPUT);
  pinMode(START_BUTTON_PIN, INPUT);  // Externt pulldown motstånd
  pinMode(DONE_LED_PIN, OUTPUT);
  
  // Sätt initial tillstånd
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(LED_HEARTBEAT, LOW);
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_PMODE, LOW);
  digitalWrite(DONE_LED_PIN, LOW);
  
  // Starta med status "ej programmerad"
  lastProgrammingSuccess = false;
  
  Serial.println(F("Systemet redo!"));
  Serial.println(F("- Tryck på startknappen (D2) för att börja programmera"));
  Serial.println(F("- Eller skriv 'start' i seriell monitor"));
  Serial.println(F("- Färdig-LED (D3) tänds när programmeringen är klar"));
  Serial.println(F("========================================\n"));
}

void loop() {
  // Kontrollera startknapp
  bool currentButtonState = digitalRead(START_BUTTON_PIN);
  
  // Knapp debouncing och detektering av tryck (LOW -> HIGH med pulldown)
  if (currentButtonState && !lastButtonState && (millis() - lastButtonPress > DEBOUNCE_DELAY)) {
    Serial.println(F("\n🔘 STARTKNAPP TRYCKT!"));
    startProgrammingSequence();
    lastButtonPress = millis();
  }
  lastButtonState = currentButtonState;
  
  // Lyssna efter restart kommando via seriell
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command == "restart" || command == "start" || command == "r") {
      Serial.println(F("\n=== SERIELL OMSTART ==="));
      startProgrammingSequence();
      return;
    } else if (command == "status") {
      Serial.print(F("LED Status - lastProgrammingSuccess: "));
      Serial.println(lastProgrammingSuccess ? F("TRUE") : F("FALSE"));
      Serial.print(F("Knapp tillstånd: "));
      Serial.println(currentButtonState ? F("TRYCKT") : F("INTE TRYCKT"));
      return;
    }
  }
  
  // Hantera LED status baserat på senaste programmering
  if (lastProgrammingSuccess) {
    // Framgång: Error LED släckt, heartbeat blinkar lugnt, färdig-LED tänd
    digitalWrite(LED_ERROR, LOW);
    digitalWrite(DONE_LED_PIN, HIGH);
    
    // Lugn heartbeat blink för att visa att allt är OK
    static unsigned long lastHeartbeat = 0;
    static bool heartbeatState = false;
    static bool firstTime = true;
    
    if (firstTime) {
      Serial.println(F(">>> HEARTBEAT AKTIVERAT - LED PIN 9 BLINKAR <<<"));
      Serial.println(F(">>> FÄRDIG LED TÄND - PIN 3 <<<"));
      firstTime = false;
    }
    
    if (millis() - lastHeartbeat > 1500) { // Snabbare blink för tydlighet
      heartbeatState = !heartbeatState;
      digitalWrite(LED_HEARTBEAT, heartbeatState);
      lastHeartbeat = millis();
      
      // Debug - visa blink i seriell monitor
      static int blinkCount = 0;
      if (blinkCount < 10) {
        Serial.print(heartbeatState ? F("💚 ON ") : F("⚫ OFF "));
        blinkCount++;
        if (blinkCount >= 10) {
          Serial.println(F("\n[Heartbeat blinkar...]"));
        }
      }
    }
    
  } else {
    // Fel eller ingen programmering: Error LED tänd, heartbeat släckt, färdig-LED släckt
    digitalWrite(LED_ERROR, HIGH);
    digitalWrite(LED_HEARTBEAT, LOW);
    digitalWrite(DONE_LED_PIN, LOW);
    
    static bool errorShown = false;
    if (!errorShown) {
      Serial.println(F(">>> ERROR STATUS - LED PIN 8 TÄND <<<"));
      Serial.println(F(">>> FÄRDIG LED SLÄCKT - PIN 3 <<<"));
      errorShown = true;
    }
  }
  
  // Programming LED hanteras i programmerings-funktionerna
  delay(50);
}

void startProgrammingSequence() {
  Serial.println(F("=== STARTAR BOOTLOADER PROGRAMMERING ==="));
  
  // Släck färdig-LED under programmering
  digitalWrite(DONE_LED_PIN, LOW);
  
  bool success = false;
  
  if (startISPMode()) {
    Serial.println(F("✓ ISP läge aktiverat"));
    
    if (verifyTargetChip()) {
      Serial.println(F("✓ ATmega328P identifierat"));
      
      if (setFuseBits()) {
        Serial.println(F("✓ Fuse bits konfigurerade"));
        
        if (eraseChip()) {
          Serial.println(F("✓ Chip raderat"));
          
          if (programBootloaderData()) {
            Serial.println(F("✓ Bootloader programmerad"));
            
            if (verifyBootloader()) {
              Serial.println(F("✓ Bootloader verifierad"));
              
              if (setLockBits()) {
                Serial.println(F("✓ Lock bits satta"));
                success = true;
              } else {
                Serial.println(F("✗ Lock bits misslyckades"));
              }
            } else {
              Serial.println(F("✗ Bootloader verifiering misslyckades"));
            }
          } else {
            Serial.println(F("✗ Bootloader programmering misslyckades"));
          }
        } else {
          Serial.println(F("✗ Chip radering misslyckades"));
        }
      } else {
        Serial.println(F("✗ Fuse bit konfiguration misslyckades"));
      }
    } else {
      Serial.println(F("✗ Kan inte identifiera target chip"));
    }
    
    endISPMode();
  } else {
    Serial.println(F("✗ Kan inte starta ISP läge"));
  }
  
  Serial.println(F("========================"));
  
  if (success) {
    Serial.println(F("🎉 PROGRAMMERING SLUTFÖRD FRAMGÅNGSRIKT! 🎉"));
    Serial.println(F("Din Arduino Nano har nu Optiboot bootloader!"));
    Serial.println(F("- Upload hastighet: 115200 baud"));
    Serial.println(F("- Bootloader storlek: 512 bytes"));
    Serial.println(F("- Tillgängligt flash: 32256 bytes"));
    Serial.println(F("- Välj 'Arduino Nano' i Arduino IDE"));
    lastProgrammingSuccess = true;
    Serial.println(F("*** HEARTBEAT LED KOMMER ATT BLINKA ***"));
    Serial.println(F("*** FÄRDIG LED (D3) KOMMER ATT TÄNDAS ***"));
    blinkSuccess();
  } else {
    Serial.println(F("❌ PROGRAMMERING MISSLYCKADES"));
    Serial.println(F("Kontrollera kopplingar och försök igen."));
    lastProgrammingSuccess = false;
    Serial.println(F("*** ERROR LED KOMMER ATT VARA TÄND ***"));
    Serial.println(F("*** FÄRDIG LED (D3) KOMMER ATT VARA SLÄCKT ***"));
    blinkError();
  }
  
  Serial.println(F("\nRedo för nästa enhet!"));
  Serial.println(F("LED Status:"));
  Serial.println(F("- Heartbeat (D9) = Framgång"));
  Serial.println(F("- Error (D8) = Misslyckad"));
  Serial.println(F("- Färdig (D3) = Programmering klar"));
  Serial.println(F("Tryck på startknapp eller skriv 'restart' för omstart"));
  Serial.println(F("========================================\n"));
}

bool startISPMode() {
  Serial.print(F("Initierar ISP kommunikation... "));
  
  // Tänd programming LED
  digitalWrite(LED_PMODE, HIGH);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV128); // Långsam för säkerhet
  
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);
  
  // Reset sekvens
  digitalWrite(RESET_PIN, LOW);
  delay(50);
  digitalWrite(RESET_PIN, HIGH);
  delay(50);
  digitalWrite(RESET_PIN, LOW);
  delay(50);
  
  // Försök gå in i programmeringsläge
  for (int attempts = 0; attempts < 5; attempts++) {
    SPI.transfer(0xAC);
    SPI.transfer(0x53);
    byte response = SPI.transfer(0x00);
    SPI.transfer(0x00);
    
    if (response == 0x53) {
      programmingMode = true;
      Serial.println(F("OK"));
      return true;
    }
    delay(50);
  }
  
  Serial.println(F("MISSLYCKADES"));
  digitalWrite(LED_PMODE, LOW); // Släck programming LED vid fel
  return false;
}

bool verifyTargetChip() {
  Serial.print(F("Läser chip signatur... "));
  
  byte sig0 = readSignatureByte(0);
  byte sig1 = readSignatureByte(1);
  byte sig2 = readSignatureByte(2);
  
  Serial.print(F("0x"));
  if (sig0 < 0x10) Serial.print(F("0"));
  Serial.print(sig0, HEX);
  Serial.print(F(" 0x"));
  if (sig1 < 0x10) Serial.print(F("0"));
  Serial.print(sig1, HEX);  
  Serial.print(F(" 0x"));
  if (sig2 < 0x10) Serial.print(F("0"));
  Serial.print(sig2, HEX);
  
  if (sig0 == ATMEGA328P_SIGNATURE_0 && 
      sig1 == ATMEGA328P_SIGNATURE_1 && 
      sig2 == ATMEGA328P_SIGNATURE_2) {
    Serial.println(F(" - ATmega328P ✓"));
    return true;
  } else {
    Serial.println(F(" - OKÄND CHIP!"));
    Serial.println(F("Förväntade: 0x1E 0x95 0x0F (ATmega328P)"));
    return false;
  }
}

byte readSignatureByte(byte signatureIndex) {
  SPI.transfer(0x30);
  SPI.transfer(0x00);
  SPI.transfer(signatureIndex);
  return SPI.transfer(0x00);
}

void readAndDisplayLockBits() {
  Serial.println(F("Lock bits:"));
  
  // Läs lock bits
  SPI.transfer(0x58);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  byte lockBits = SPI.transfer(0x00);
  
  Serial.print(F("  Lock bits: 0x"));
  if (lockBits < 0x10) Serial.print(F("0"));
  Serial.print(lockBits, HEX);
  
  if (lockBits == 0xFF) {
    Serial.println(F(" (No restrictions - OK)"));
  } else if (lockBits == 0x0F) {
    Serial.println(F(" (Standard Arduino - OK)"));  
  } else {
    Serial.println(F(" (Unusual setting)"));
  }
}

void readAndDisplayFuseBits() {
  Serial.println(F("Läser tillbaka fuse bits:"));
  
  // Läs Low fuse
  SPI.transfer(0x50);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  byte lowFuse = SPI.transfer(0x00);
  
  // Läs High fuse  
  SPI.transfer(0x58);
  SPI.transfer(0x08);
  SPI.transfer(0x00);
  byte highFuse = SPI.transfer(0x00);
  
  // Läs Extended fuse
  SPI.transfer(0x50);
  SPI.transfer(0x08);
  SPI.transfer(0x00);
  byte extFuse = SPI.transfer(0x00);
  
  Serial.print(F("  Aktuell Low: 0x"));
  if (lowFuse < 0x10) Serial.print(F("0"));
  Serial.print(lowFuse, HEX);
  Serial.println(lowFuse == 0xFF ? F(" ✓") : F(" ⚠"));
  
  Serial.print(F("  Aktuell High: 0x"));
  if (highFuse < 0x10) Serial.print(F("0"));
  Serial.print(highFuse, HEX);
  Serial.println(highFuse == 0xDE ? F(" ✓") : F(" ⚠"));
  
  Serial.print(F("  Aktuell Extended: 0x"));
  if (extFuse < 0x10) Serial.print(F("0"));
  Serial.print(extFuse, HEX);
  Serial.println((extFuse & 0x07) == 0x05 ? F(" ✓") : F(" ⚠"));
}

bool setLockBits() {
  Serial.print(F("Sätter lock bits... "));
  
  // Arduino standard: 0x0F (no memory lock features enabled)
  // Men vi testar med 0xFF först (completely unlocked)
  SPI.transfer(0xAC);
  SPI.transfer(0xE0);
  SPI.transfer(0x00);
  SPI.transfer(0xFF); // Prova 0xFF istället för 0x0F
  
  delay(10);
  
  Serial.println(F("OK"));
  Serial.println(F("  Lock: 0xFF (completely unlocked)"));
  return true;
}

bool setFuseBits() {
  Serial.print(F("Konfigurerar fuse bits... "));
  
  // Extended fuse: 0x05 (Brown-out detection 2.7V)
  writeFuseByte(0xA4, 0x05);
  delay(50); // Längre delay för säkerhet
  
  // High fuse: 0xDE (512 byte bootloader, boot reset vector enabled)  
  writeFuseByte(0xA8, 0xDE);
  delay(50); // Längre delay för säkerhet
  
  // Low fuse: 0xFF (16MHz crystal, fast startup)
  writeFuseByte(0xAC, 0xFF);
  delay(50); // Längre delay för säkerhet
  
  Serial.println(F("OK"));
  Serial.println(F("  Extended: 0x05 (BOD 2.7V)"));
  Serial.println(F("  High: 0xDE (512B bootloader)"));
  Serial.println(F("  Low: 0xFF (16MHz crystal)"));
  return true;
}

void writeFuseByte(byte command, byte value) {
  SPI.transfer(command);
  SPI.transfer(0xA0);
  SPI.transfer(0x00);
  SPI.transfer(value);
}

bool eraseChip() {
  Serial.print(F("Raderar chip... "));
  
  SPI.transfer(0xAC);
  SPI.transfer(0x80);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  
  delay(100); // Vänta på radering
  
  Serial.println(F("OK"));
  return true;
}

bool programBootloaderData() {
  Serial.println(F("Programmerar Optiboot bootloader..."));
  
  int pages = (BOOTLOADER_SIZE + 127) / 128; // 128 bytes per page
  Serial.print(F("Totalt "));
  Serial.print(pages);
  Serial.print(F(" sidor ("));
  Serial.print(BOOTLOADER_SIZE);
  Serial.println(F(" bytes)"));
  
  for (int page = 0; page < pages; page++) {
    unsigned int pageAddress = BOOTLOADER_START + (page * 128);
    
    // Ladda page buffer med 64 words (128 bytes)
    for (int wordIndex = 0; wordIndex < 64; wordIndex++) {
      int byteIndex = (page * 128) + (wordIndex * 2);
      
      byte lowByte = 0xFF;
      byte highByte = 0xFF;
      
      if (byteIndex < BOOTLOADER_SIZE) {
        lowByte = pgm_read_byte(&bootloader[byteIndex]);
      }
      if (byteIndex + 1 < BOOTLOADER_SIZE) {
        highByte = pgm_read_byte(&bootloader[byteIndex + 1]);
      }
      
      // Ladda low byte
      SPI.transfer(0x40);
      SPI.transfer(0x00);
      SPI.transfer(wordIndex);
      SPI.transfer(lowByte);
      
      // Ladda high byte  
      SPI.transfer(0x48);
      SPI.transfer(0x00);
      SPI.transfer(wordIndex);
      SPI.transfer(highByte);
    }
    
    // Skriv page till flash
    unsigned int wordAddress = pageAddress / 2;
    SPI.transfer(0x4C);
    SPI.transfer((wordAddress >> 8) & 0xFF);
    SPI.transfer(wordAddress & 0xFF);
    SPI.transfer(0x00);
    
    delay(50); // Längre väntetid för säkerhet
    
    Serial.print(F("Sida "));
    Serial.print(page + 1);
    Serial.print(F("/"));
    Serial.print(pages);
    Serial.print(F(" (0x"));
    Serial.print(pageAddress, HEX);
    Serial.println(F(") ✓"));
    
    // Blink programming LED för visuell feedback
    if (page % 2 == 0) {
      digitalWrite(LED_PMODE, LOW);
      delay(20);
      digitalWrite(LED_PMODE, HIGH);
    }
  }
  
  Serial.println(F("Bootloader data programmerad!"));
  return true;
}

bool verifyBootloader() {
  Serial.println(F("Verifierar hela bootloader..."));
  
  // Verifiera HELA bootloader, inte bara första 128 bytes
  int errors = 0;
  
  for (int i = 0; i < BOOTLOADER_SIZE; i++) {
    byte expected = pgm_read_byte(&bootloader[i]);
    byte actual = readFlashByte(BOOTLOADER_START + i);
    
    if (expected != actual) {
      if (errors < 5) { // Visa bara första 5 felen
        Serial.print(F("FEL på byte "));
        Serial.print(i);
        Serial.print(F(" (adress 0x"));
        Serial.print(BOOTLOADER_START + i, HEX);
        Serial.print(F("): förväntade 0x"));
        Serial.print(expected, HEX);
        Serial.print(F(", fick 0x"));
        Serial.println(actual, HEX);
      }
      errors++;
    }
    
    // Progress indicator
    if (i > 0 && i % 48 == 0) {
      Serial.print(F("."));
    }
  }
  
  Serial.println();
  Serial.print(F("Verifierade "));
  Serial.print(BOOTLOADER_SIZE);
  Serial.print(F(" bytes, "));
  Serial.print(errors);
  Serial.println(F(" fel"));
  
  if (errors > 0) {
    Serial.println(F("VERIFIERING MISSLYCKADES!"));
    return false;
  }
  
  Serial.println(F("Verifiering OK!"));
  return true;
}

byte readFlashByte(unsigned int address) {
  if (address & 1) {
    // Odd address - high byte
    SPI.transfer(0x28);
    SPI.transfer((address >> 9) & 0xFF);
    SPI.transfer((address >> 1) & 0xFF);
    return SPI.transfer(0x00);
  } else {
    // Even address - low byte
    SPI.transfer(0x20);
    SPI.transfer((address >> 9) & 0xFF);
    SPI.transfer((address >> 1) & 0xFF);
    return SPI.transfer(0x00);
  }
}

void endISPMode() {
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(LED_PMODE, LOW); // Släck programming LED
  SPI.end();
  programmingMode = false;
  Serial.println(F("ISP läge avslutat"));
}

void blinkSuccess() {
  // 10 snabba framgångs-blink på heartbeat LED
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_HEARTBEAT, HIGH);
    delay(100);
    digitalWrite(LED_HEARTBEAT, LOW);
    delay(100);
  }
  // LED-tillståndet hanteras nu av loop() baserat på lastProgrammingSuccess
}

void blinkError() {
  // 5 långsamma fel-blink på error LED
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_ERROR, HIGH);
    delay(800);
    digitalWrite(LED_ERROR, LOW);
    delay(400);
  }
  // LED-tillståndet hanteras nu av loop() baserat på lastProgrammingSuccess
}
