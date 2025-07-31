#include "BluetoothSerial.h"
#include "FS.h"
#include "SD_MMC.h"  
#include "SPI.h"
#include <ESP32Servo.h>

// ===== CONFIGURAZIONE PIN =====
#define TRIG_PIN 0        // Trigger sensore ultrasuoni - genera impulso 10µs
#define ECHO_PIN 33       // Echo sensore ultrasuoni - riceve risposta temporizzata
#define SERVO_PIN 13      // PWM per controllo servo motore sbarra

// ===== CONFIGURAZIONE SD_MMC =====
#define SD_MMC_CMD 15     // Linea comando SD - protocollo MMC
#define SD_MMC_CLK 14     // Clock sincronizzazione dati
#define SD_MMC_D0 2       // Linea dati 0 (modalità 1-bit)

// ===== CONFIGURAZIONE SENSORE =====
#define DISTANCE_THRESHOLD 20    // Soglia rilevamento veicolo (cm)
#define MIN_DETECTION_TIME 3000  // Filtro anti-rimbalzo per conferma presenza (ms)
#define MAX_DISTANCE 400         // Limite massimo lettura valida sensore (cm)

// ===== CONFIGURAZIONE DIAGNOSTICA SENSORE =====
#define SENSOR_TEST_SAMPLES 10      // Campioni per test affidabilità
#define SENSOR_ERROR_THRESHOLD 5    // Errori consecutivi prima di segnalare guasto
#define SENSOR_TIMEOUT_US 30000     // Timeout lettura ultrasuoni (30ms)
#define SENSOR_MIN_DISTANCE 2       // Distanza minima fisica valida (cm)
#define SENSOR_STABILITY_SAMPLES 5  // Campioni per calcolo stabilità
#define SENSOR_VARIANCE_THRESHOLD 10 // Varianza massima per sensore stabile (cm)

// ===== CONFIGURAZIONE SERVO =====
#define SERVO_CLOSED_ANGLE 0     // Posizione sbarra chiusa (gradi)
#define SERVO_OPEN_ANGLE 90      // Posizione sbarra aperta (gradi)
#define GATE_OPEN_DURATION 5000  // Tempo massimo apertura (sicurezza)
#define GATE_DELAY_CLOSE 5000    // Ritardo chiusura dopo uscita veicolo

// ===== CONFIGURAZIONE SICUREZZA =====
#define MAX_AUTH_ATTEMPTS 3      // Limite tentativi per sessione (anti-brute force)
#define AUTH_TIMEOUT 10000       // Timeout autenticazione utente (ms)

BluetoothSerial SerialBT;
Servo gateServo;

// ===== VARIABILI STATO SISTEMA =====
bool vehiclePresent = false;           // Flag presenza veicolo
bool authenticationInProgress = false; // Flag sessione autenticazione attiva
bool gateIsOpen = false;              // Stato attuale sbarra
unsigned long vehicleDetectedTime = 0; // Timestamp primo rilevamento
unsigned long vehicleLeftTime = 0;     // Timestamp uscita veicolo
unsigned long gateOpenedTime = 0;      // Timestamp apertura sbarra
unsigned long authStartTime = 0;       // Timestamp inizio autenticazione
String currentUser = "";               // Username sessione corrente
int authAttempts = 0;                 // Contatore tentativi falliti
int messageCount = 0;                 // Statistiche messaggi Bluetooth
bool sdCardAvailable = false;         // Flag disponibilità storage persistente

// ===== VARIABILI DIAGNOSTICA SENSORE =====
bool sensorHealthy = true;            // Stato generale sensore
int consecutiveErrors = 0;            // Contatore errori sequenziali
unsigned long totalReadings = 0;      // Totale letture effettuate
unsigned long errorReadings = 0;      // Totale letture errate
unsigned long lastSensorTest = 0;     // Timestamp ultimo test diagnostico
float lastValidDistance = -1;         // Ultima lettura valida per reference
bool sensorCalibrated = false;        // Flag calibrazione completata

// ===== STRUTTURA STATISTICHE SENSORE =====
struct SensorStats {
  unsigned long totalMeasurements;    // Contatore misurazioni totali
  unsigned long errorCount;           // Contatore errori generali
  unsigned long timeoutCount;         // Contatore timeout specifici
  float minDistance;                  // Distanza minima rilevata
  float maxDistance;                  // Distanza massima rilevata
  float avgDistance;                  // Media mobile delle distanze
  unsigned long lastResetTime;       // Timestamp ultimo reset statistiche
};
SensorStats sensorStats = {0, 0, 0, 999.0, 0.0, 0.0, 0};

// ===== DATABASE FALLBACK IN MEMORIA =====
struct MemoryUser {
  String username;
  String password;
};

// Database hardcoded per funzionamento senza SD
MemoryUser memoryUsers[] = {
  {"user1", "user1"}, 
  {"user2", "user2"}
};

const int NUM_MEMORY_USERS = sizeof(memoryUsers) / sizeof(memoryUsers[0]);

// ===== MACCHINA A STATI SISTEMA =====
enum SystemState {
  IDLE_STATE,           // Sistema in attesa
  VEHICLE_DETECTED,     // Veicolo rilevato, verifica in corso
  REQUEST_USERNAME,     // Richiesta credenziali utente
  REQUEST_PASSWORD,     // Richiesta password
  ACCESS_GRANTED,       // Autenticazione riuscita
  ACCESS_DENIED,        // Autenticazione fallita
  GATE_OPERATING,       // Sbarra in movimento/aperta
  GATE_WAITING_CLOSE,   // Countdown per chiusura automatica
  SENSOR_ERROR          // Errore critico sensore
};

SystemState currentSystemState = IDLE_STATE;

void setup() {
  Serial.begin(115200);
  delay(1000);
 
  // Banner informativo sistema
  Serial.println("=================================");
  Serial.println("  SISTEMA AUTENTIFICAZIONE ACCESSO");
  Serial.println("  Progetto con ESP32");
  Serial.println("=================================");
 
  // ===== INIZIALIZZAZIONE BLUETOOTH =====
  SerialBT.begin("ESP32_Controllo_Accessi"); // Nome device visibile
  Serial.println("✓ Bluetooth attivo: ESP32_Controllo_Accessi");
 
  // ===== CONFIGURAZIONE SENSORE ULTRASUONI =====
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW); // Stato iniziale trigger basso
  Serial.println("✓ Sensore ultrasuoni configurato (soglia: " + String(DISTANCE_THRESHOLD) + "cm)");
 
  // ===== TEST INIZIALE SENSORE =====
  Serial.println("🔍 Esecuzione test iniziale sensore...");
  bool sensorOK = performSensorDiagnostic();
  if (sensorOK) {
    Serial.println("✅ Sensore ultrasuoni FUNZIONANTE");
    sensorHealthy = true;
  } else {
    Serial.println("⚠️ ATTENZIONE: Possibili problemi con sensore ultrasuoni!");
    sensorHealthy = false;
  }
 
  // ===== INIZIALIZZAZIONE SERVO =====
  gateServo.attach(SERVO_PIN);
  gateServo.write(SERVO_CLOSED_ANGLE); // Posizione iniziale chiusa
  Serial.println("✓ Servo sbarra posizionato (chiuso)");
 
  // ===== INIZIALIZZAZIONE MICROSD =====
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0); // Configurazione pin MMC
  if(!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    // Fallback: sistema continua senza storage persistente
    Serial.println("⚠️ ATTENZIONE: MicroSD non rilevata!");
    Serial.println("  Sistema funzionerà con database in memoria");
    Serial.println("  Le credenziali NON saranno salvate permanentemente");
    sdCardAvailable = false;
  } else {
    sdCardAvailable = true;
    Serial.println("✓ MicroSD inizializzata correttamente");
   
    // Verifica tipo scheda per debugging
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
      Serial.println("⚠️ No SD_MMC card attached");
      sdCardAvailable = false;
    } else {
      Serial.println("✓ Tipo scheda SD: " + String(cardType == CARD_MMC ? "MMC" :
                     cardType == CARD_SD ? "SDSC" :
                     cardType == CARD_SDHC ? "SDHC" : "UNKNOWN"));
    }
  }
 
  // ===== SETUP DATABASE UTENTI =====
  initializeDatabase(sdCardAvailable);
 
  // ===== INIZIALIZZAZIONE STATISTICHE =====
  sensorStats.lastResetTime = millis();
  sensorStats.minDistance = 999.0; // Valore iniziale alto per rilevare vero minimo
 
  Serial.println("=================================");
  Serial.println("SISTEMA PRONTO!");
  Serial.println("In attesa di veicoli...");
  Serial.println("Comandi Bluetooth disponibili:");
  Serial.println("- help, test, users, stats, ping");
  Serial.println("- sensor, calibrate, reset-sensor");
  Serial.println("=================================");
 
  // Messaggio benvenuto via Bluetooth
  SerialBT.println("SISTEMA:Bluetooth connesso - Sistema Controllo Accessi");
  SerialBT.println("SISTEMA:Diagnostica sensore: " + String(sensorHealthy ? "OK" : "ERRORE"));
  SerialBT.println("SISTEMA:Digita 'help' per comandi di test");
}

void loop() {
  // Polling continuo presenza veicolo
  checkVehicleDetection();
 
  // Gestione transizioni stati sistema
  handleSystemStates();
 
  // Processing messaggi Bluetooth (autenticazione + comandi debug)
  processBluetoothMessages();
 
  // Controllo timer e timeout critici
  checkTimeouts();
 
  // Diagnostica periodica sensore (ogni 30s)
  if (millis() - lastSensorTest > 30000) {
    checkSensorHealth();
    lastSensorTest = millis();
  }
 
  delay(50); // Pausa per ottimizzazione CPU e stabilità letture
}

// ===== FUNZIONI SENSORE ULTRASUONI =====
void checkVehicleDetection() {
  long distance = measureDistance(); // Lettura distanza corrente
 
  updateSensorStats(distance); // Aggiornamento statistiche continue
 
  // Filtro letture valide (elimina spike e errori)
  if (distance > 0 && distance < MAX_DISTANCE) {
   
    if (distance <= DISTANCE_THRESHOLD) {
      // VEICOLO RILEVATO
      if (!vehiclePresent) {
        vehiclePresent = true;
        vehicleDetectedTime = millis(); // Timestamp primo rilevamento
        vehicleLeftTime = 0; // Reset timer uscita
        Serial.println("📡 Veicolo rilevato a " + String(distance) + " cm");
       
        // Reset contatore errori su lettura valida
        consecutiveErrors = 0;
        if (!sensorHealthy && consecutiveErrors == 0) {
          Serial.println("✅ Sensore sembra essere tornato funzionante");
          sensorHealthy = true;
          if (currentSystemState == SENSOR_ERROR) {
            currentSystemState = IDLE_STATE; // Recupero automatico
          }
        }
      }
     
      // Controllo persistenza presenza (filtro anti-rimbalzo)
      if (millis() - vehicleDetectedTime > MIN_DETECTION_TIME &&
          currentSystemState == IDLE_STATE && sensorHealthy) {
        startAuthenticationProcess(); // Avvia sequenza autenticazione
      }
    } else {
      // NESSUN VEICOLO PRESENTE
      if (vehiclePresent) {
        vehiclePresent = false;
        vehicleLeftTime = millis(); // Timestamp uscita per timer chiusura
        Serial.println("📡 Veicolo non più presente - avvio timer chiusura");
       
        // Gestione transizione stato per chiusura ritardata
        if (gateIsOpen && currentSystemState == GATE_OPERATING) {
          currentSystemState = GATE_WAITING_CLOSE;
        }
        // Reset solo se non in attesa di chiusura e non in autenticazione
        else if (currentSystemState != GATE_WAITING_CLOSE && !authenticationInProgress) {
          resetToIdle();
        }
      }
    }
  } else {
    // LETTURA ERRATA - gestione errori
    consecutiveErrors++;
    if (consecutiveErrors >= SENSOR_ERROR_THRESHOLD) {
      if (sensorHealthy) {
        Serial.println("❌ ERRORE SENSORE: Troppi errori consecutivi!");
        SerialBT.println("SISTEMA:⚠️ ERRORE SENSORE RILEVATO!");
        sensorHealthy = false;
        currentSystemState = SENSOR_ERROR; // Blocca sistema per sicurezza
      }
    }
  }
}

long measureDistance() {
  totalReadings++; // Contatore statistiche
 
  // Generazione impulso trigger (protocollo HC-SR04)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);  // Pausa pre-impulso
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); // Impulso trigger 10µs
  digitalWrite(TRIG_PIN, LOW);
 
  // Misurazione durata echo con timeout per evitare hang
  long duration = pulseIn(ECHO_PIN, HIGH, SENSOR_TIMEOUT_US);
 
  if (duration == 0) {
    // Timeout - sensore non risponde
    errorReadings++;
    sensorStats.timeoutCount++;
    return -1; // Codice errore timeout
  }
 
  // Conversione tempo in distanza: v_suono = 343m/s
  // distanza = (durata * velocità) / 2 (andata+ritorno)
  long distance = duration * 0.034 / 2;
 
  // Validazione range fisico sensore
  if (distance < SENSOR_MIN_DISTANCE || distance > MAX_DISTANCE) {
    errorReadings++;
    sensorStats.errorCount++;
    return -2; // Codice errore range
  }
 
  lastValidDistance = distance; // Salva per riferimento
  return distance;
}

// ===== FUNZIONI DIAGNOSTICA SENSORE =====
bool performSensorDiagnostic() {
  Serial.println("🔍 Esecuzione diagnostica completa sensore...");
  SerialBT.println("SISTEMA:🔍 Test sensore in corso...");
 
  // Contatori per analisi statistiche
  int validReadings = 0;
  int timeouts = 0;
  int invalidReadings = 0;
  float readings[SENSOR_TEST_SAMPLES];
 
  // Campionamento multiplo per affidabilità
  for (int i = 0; i < SENSOR_TEST_SAMPLES; i++) {
    long distance = measureDistance();
   
    // Classificazione risultato lettura
    if (distance == -1) {
      timeouts++;
    } else if (distance == -2) {
      invalidReadings++;
    } else {
      readings[validReadings] = distance;
      validReadings++;
    }
   
    delay(100); // Pausa tra campioni per stabilità
  }
 
  // Calcolo tasso successo
  float successRate = (float)validReadings / SENSOR_TEST_SAMPLES * 100;
 
  // Report diagnostica
  Serial.println("📊 Risultati diagnostica:");
  Serial.println("   Letture valide: " + String(validReadings) + "/" + String(SENSOR_TEST_SAMPLES));
  Serial.println("   Timeout: " + String(timeouts));
  Serial.println("   Letture invalide: " + String(invalidReadings));
  Serial.println("   Tasso successo: " + String(successRate) + "%");
 
  SerialBT.println("SISTEMA:📊 Letture valide: " + String(validReadings) + "/" + String(SENSOR_TEST_SAMPLES));
  SerialBT.println("SISTEMA:📊 Tasso successo: " + String(successRate) + "%");
 
  // Test stabilità se sufficienti dati
  if (validReadings >= SENSOR_STABILITY_SAMPLES) {
    float variance = calculateVariance(readings, validReadings);
    Serial.println("   Varianza: " + String(variance) + " cm");
    SerialBT.println("SISTEMA:📊 Stabilità: " + String(variance < SENSOR_VARIANCE_THRESHOLD ? "BUONA" : "INSTABILE"));
   
    if (variance > SENSOR_VARIANCE_THRESHOLD) {
      Serial.println("⚠️ Sensore instabile - varianza troppo alta");
      return false; // Fallisce test stabilità
    }
  }
 
  // Soglia accettabilità: 70% successo minimo
  return successRate >= 70.0;
}

float calculateVariance(float readings[], int count) {
  if (count < 2) return 0; // Serve almeno 2 campioni
 
  // Calcolo media aritmetica
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += readings[i];
  }
  float mean = sum / count;
 
  // Calcolo deviazione standard
  float variance = 0;
  for (int i = 0; i < count; i++) {
    variance += pow(readings[i] - mean, 2);
  }
 
  return sqrt(variance / (count - 1)); // Deviazione standard campionaria
}

void checkSensorHealth() {
  // Controllo tasso errore su lungo periodo
  if (totalReadings > 100) { // Servono dati sufficienti
    float errorRate = (float)errorReadings / totalReadings * 100;
   
    if (errorRate > 30.0 && sensorHealthy) {
      Serial.println("⚠️ Tasso errore sensore elevato: " + String(errorRate) + "%");
      SerialBT.println("SISTEMA:⚠️ Sensore instabile - tasso errore: " + String(errorRate) + "%");
    }
  }
}

void updateSensorStats(long distance) {
  sensorStats.totalMeasurements++;
 
  if (distance > 0) {
    // Aggiornamento range min/max
    if (distance < sensorStats.minDistance) {
      sensorStats.minDistance = distance;
    }
    if (distance > sensorStats.maxDistance) {
      sensorStats.maxDistance = distance;
    }
   
    // Media mobile esponenziale (peso 90% vecchio, 10% nuovo)
    sensorStats.avgDistance = (sensorStats.avgDistance * 0.9) + (distance * 0.1);
  } else {
    sensorStats.errorCount++; // Incrementa contatore errori
  }
}

void resetSensorStats() {
  // Reset completo statistiche sensore
  sensorStats.totalMeasurements = 0;
  sensorStats.errorCount = 0;
  sensorStats.timeoutCount = 0;
  sensorStats.minDistance = 999.0;
  sensorStats.maxDistance = 0.0;
  sensorStats.avgDistance = 0.0;
  sensorStats.lastResetTime = millis();
 
  // Reset contatori globali
  totalReadings = 0;
  errorReadings = 0;
  consecutiveErrors = 0;
 
  Serial.println("🔄 Statistiche sensore resettate");
  SerialBT.println("SISTEMA:🔄 Statistiche sensore resettate");
}

// ===== GESTIONE STATI SISTEMA =====
void handleSystemStates() {
  // Switch basato su stato corrente macchina a stati
  switch (currentSystemState) {
    case IDLE_STATE:
      // Sistema in attesa - nessuna azione richiesta
      break;
     
    case VEHICLE_DETECTED:
      // Gestito in checkVehicleDetection()
      break;
     
    case REQUEST_USERNAME:
    case REQUEST_PASSWORD:
      // Gestiti in processBluetoothMessages()
      break;
     
    case ACCESS_GRANTED:
      openGate(); // Esegui apertura sbarra
      currentSystemState = GATE_OPERATING;
      break;
     
    case ACCESS_DENIED:
      delay(2000); // Pausa penalizzante prima reset
      resetToIdle();
      break;
     
    case GATE_OPERATING:
      // Timer gestito in checkTimeouts()
      break;
     
    case GATE_WAITING_CLOSE:
      // Chiusura ritardata dopo uscita veicolo
      if (vehicleLeftTime > 0 && millis() - vehicleLeftTime > GATE_DELAY_CLOSE) {
        closeGate();
      }
      break;
     
    case SENSOR_ERROR:
      // Sistema bloccato - richiede intervento manuale o reset forzato
      break;
  }
}

// ===== PROCESSO AUTENTICAZIONE =====
void startAuthenticationProcess() {
  // Verifica preliminare stato sensore
  if (!sensorHealthy) {
    Serial.println("⚠️ Autenticazione bloccata - sensore non funzionante");
    SerialBT.println("SISTEMA:⚠️ Servizio temporaneamente non disponibile");
    return;
  }
 
  // Inizializzazione sessione autenticazione
  authenticationInProgress = true;
  authStartTime = millis();
  authAttempts = 0;
  currentSystemState = REQUEST_USERNAME;
 
  Serial.println("🔐 Avvio autenticazione...");
  SerialBT.println("ACCESSO:Veicolo rilevato!");
  SerialBT.println("ACCESSO:Inserisci il tuo USERNAME:");
}

void processBluetoothMessages() {
  if (SerialBT.available()) {
    String receivedMessage = SerialBT.readStringUntil('\n');
    receivedMessage.trim(); // Rimozione spazi e caratteri controllo
   
    if (receivedMessage.length() > 0) {
      messageCount++; // Statistiche traffico Bluetooth
      Serial.println("📱 [" + String(messageCount) + "] Ricevuto: '" + receivedMessage + "'");
     
      // Routing messaggio basato su stato sistema
      if (currentSystemState == REQUEST_USERNAME || currentSystemState == REQUEST_PASSWORD) {
        // MODALITÀ AUTENTICAZIONE
        switch (currentSystemState) {
          case REQUEST_USERNAME:
            handleUsernameValidation(receivedMessage);
            break;
           
          case REQUEST_PASSWORD:
            handlePasswordValidation(receivedMessage);
            break;
        }
      }
      else {
        // MODALITÀ COMANDI DEBUG/TEST
        processTestCommand(receivedMessage);
      }
    }
  }
}

// ===== FUNZIONI BLUETOOTH TEST/DEBUG =====
void processTestCommand(String command) {
  command.toLowerCase(); // Normalizzazione case-insensitive
 
  // Routing comandi debug con pattern matching
  if (command == "help" || command == "aiuto") {
    showHelp();
  }
  else if (command == "test") {
    runSystemTest(); // Test completo tutti i componenti
  }
  else if (command == "users" || command == "utenti") {
    showUsersViaBluetooth();
  }
  else if (command == "stats" || command == "statistiche") {
    showStats(); // Statistiche sistema e performance
  }
  else if (command == "ping") {
    SerialBT.println("SISTEMA:PONG! Sistema attivo"); // Test connettività
  }
  else if (command == "status" || command == "stato") {
    showSystemStatus(); // Stato dettagliato sistema
  }
  else if (command == "sensor" || command == "sensore") {
    showSensorDiagnostics(); // Report diagnostica sensore
  }
  else if (command == "calibrate" || command == "calibra") {
    calibrateSensor(); // Procedura calibrazione sensore
  }
  else if (command == "reset-sensor" || command == "reset-sensore") {
    resetSensorStats(); // Reset statistiche sensore
  }
  else if (command == "sensor-test" || command == "test-sensore") {
    runSensorTest(); // Test completo sensore
  }
  else if (command.startsWith("login:")) {
    testLogin(command); // Test credenziali senza attivazione sbarra
  }
  else if (command == "open" && currentSystemState == IDLE_STATE) {
    // Comando debug apertura manuale
    SerialBT.println("SISTEMA:🔧 Apertura manuale sbarra (debug)");
    openGate();
    currentSystemState = GATE_OPERATING;
  }
  else if (command == "close" && gateIsOpen) {
    // Comando debug chiusura manuale
    SerialBT.println("SISTEMA:🔧 Chiusura manuale sbarra (debug)");
    closeGate();
  }
  else if (command == "force-reset" && currentSystemState == SENSOR_ERROR) {
    // Reset forzato per uscire da errore sensore
    SerialBT.println("SISTEMA:🔧 Reset forzato stato errore sensore");
    sensorHealthy = true;
    consecutiveErrors = 0;
    resetToIdle();
  }
  else {
    // Comando non riconosciuto
    if (currentSystemState == IDLE_STATE || currentSystemState == SENSOR_ERROR) {
      SerialBT.println("SISTEMA:Comando '" + command + "' non riconosciuto");
      SerialBT.println("SISTEMA:Digita 'help' per vedere i comandi disponibili");
    } else {
      SerialBT.println("ACCESSO:Sistema non in modalità test");
    }
  }
}

void showHelp() {
  // Menu completo comandi disponibili
  SerialBT.println("SISTEMA:=== COMANDI BLUETOOTH ===");
  SerialBT.println("SISTEMA:help - Mostra questo messaggio");
  SerialBT.println("SISTEMA:test - Test completo sistema");
  SerialBT.println("SISTEMA:users - Mostra utenti database");
  SerialBT.println("SISTEMA:stats - Statistiche sistema");
  SerialBT.println("SISTEMA:status - Stato attuale sistema");
  SerialBT.println("SISTEMA:ping - Test connessione");
  SerialBT.println("SISTEMA:login:user,pass - Test credenziali");
  SerialBT.println("SISTEMA:open/close - Controllo manuale sbarra");
  SerialBT.println("SISTEMA:=== COMANDI SENSORE ===");
  SerialBT.println("SISTEMA:sensor - Diagnostica sensore");
  SerialBT.println("SISTEMA:calibrate - Calibrazione sensore");
  SerialBT.println("SISTEMA:sensor-test - Test completo sensore");
  SerialBT.println("SISTEMA:reset-sensor - Reset statistiche");
  SerialBT.println("SISTEMA:force-reset - Reset stato errore");
  SerialBT.println("SISTEMA:============================");
}

void showSensorDiagnostics() {
  // Report completo stato e statistiche sensore
  SerialBT.println("SISTEMA:🔍 === DIAGNOSTICA SENSORE ===");
  SerialBT.println("SISTEMA:📊 Stato: " + String(sensorHealthy ? "FUNZIONANTE" : "ERRORE"));
  SerialBT.println("SISTEMA:📊 Letture totali: " + String(sensorStats.totalMeasurements));
  SerialBT.println("SISTEMA:📊 Errori: " + String(sensorStats.errorCount));
  SerialBT.println("SISTEMA:📊 Timeout: " + String(sensorStats.timeoutCount));
 
  // Calcolo e visualizzazione tasso errore
  if (sensorStats.totalMeasurements > 0) {
    float errorRate = (float)sensorStats.errorCount / sensorStats.totalMeasurements * 100;
    SerialBT.println("SISTEMA:📊 Tasso errore: " + String(errorRate, 1) + "%");
  }
 
  SerialBT.println("SISTEMA:📊 Errori consecutivi: " + String(consecutiveErrors));
  SerialBT.println("SISTEMA:📊 Distanza min: " + String(sensorStats.minDistance, 1) + " cm");
  SerialBT.println("SISTEMA:📊 Distanza max: " + String(sensorStats.maxDistance, 1) + " cm");
  SerialBT.println("SISTEMA:📊 Distanza media: " + String(sensorStats.avgDistance, 1) + " cm");
 
  if (lastValidDistance > 0) {
    SerialBT.println("SISTEMA:📊 Ultima lettura: " + String(lastValidDistance) + " cm");
  }
 
  // Calcolo e visualizzazione uptime
  unsigned long uptime = (millis() - sensorStats.lastResetTime) / 1000;
  SerialBT.println("SISTEMA:📊 Uptime sensore: " + String(uptime) + "s");
  SerialBT.println("SISTEMA:================================");
}

void calibrateSensor() {
  SerialBT.println("SISTEMA:🎯 Avvio calibrazione sensore...");
  SerialBT.println("SISTEMA:Assicurati che davanti al sensore sia libero");
  delay(3000); // Tempo per preparazione ambiente
 
  // Array per raccolta dati calibrazione
  float calibrationReadings[10];
  int validCalibrations = 0;
 
  // Campionamento ambiente libero per stabilire baseline
  for (int i = 0; i < 10; i++) {
    long distance = measureDistance();
    if (distance > 0 && distance < MAX_DISTANCE) {
      calibrationReadings[validCalibrations] = distance;
      validCalibrations++;
    }
    delay(200); // Pausa tra campioni
    SerialBT.println("SISTEMA:Campione " + String(i+1) + "/10...");
  }
 
  if (validCalibrations >= 7) { // Almeno 70% campioni validi
    // Calcolo distanza media ambiente libero
    float sum = 0;
    for (int i = 0; i < validCalibrations; i++) {
      sum += calibrationReadings[i];
    }
    float avgDistance = sum / validCalibrations;
   
    SerialBT.println("SISTEMA:✅ Calibrazione completata!");
    SerialBT.println("SISTEMA:📏 Distanza ambiente libero: " + String(avgDistance, 1) + " cm");
    SerialBT.println("SISTEMA:📏 Soglia rilevamento: " + String(DISTANCE_THRESHOLD) + " cm");
   
    sensorCalibrated = true;
    resetSensorStats(); // Reset statistiche post-calibrazione
  } else {
    SerialBT.println("SISTEMA:❌ Calibrazione fallita!");
    SerialBT.println("SISTEMA:Troppi errori di lettura sensore");
  }
}

void runSensorTest() {
  SerialBT.println("SISTEMA:🧪 TEST SENSORE COMPLETO...");
 
  // Backup contatore errori per ripristino post-test
  int oldConsecutiveErrors = consecutiveErrors;
  consecutiveErrors = 0;
 
  bool testResult = performSensorDiagnostic(); // Esegui diagnostica completa
 
  if (testResult) {
    SerialBT.println("SISTEMA:✅ SENSORE: FUNZIONANTE");
    sensorHealthy = true;
    // Recupero automatico da stato errore se test OK
    if (currentSystemState == SENSOR_ERROR) {
      currentSystemState = IDLE_STATE;
      SerialBT.println("SISTEMA:🔄 Sistema ripristinato da errore sensore");
    }
  } else {
    SerialBT.println("SISTEMA:❌ SENSORE: PROBLEMI RILEVATI");
    SerialBT.println("SISTEMA:Controllare collegamenti e ostacoli");
    sensorHealthy = false;
  }
 
  // Ripristino contatore se test fallisce ma era pulito prima
  if (oldConsecutiveErrors == 0 && !testResult) {
    consecutiveErrors = oldConsecutiveErrors;
  }
}

void runSystemTest() {
  SerialBT.println("SISTEMA:🧪 TEST SISTEMA COMPLETO...");
  delay(500);
 
  // Test sensore ultrasuoni
  SerialBT.println("SISTEMA:📡 Test sensore ultrasuoni...");
  long dist = measureDistance();
  SerialBT.println("SISTEMA:📡 Distanza attuale: " + String(dist) + "cm");
  SerialBT.println("SISTEMA:📡 Stato sensore: " + String(sensorHealthy ? "OK" : "ERRORE"));
  delay(1000);
 
  // Verifica database utenti
  SerialBT.println("SISTEMA:💾 Database: " + String(countUsers()) + " utenti (" +
                   String(sdCardAvailable ? "SD" : "memoria") + ")");
  delay(1000);
 
  // Test servo motore con movimento completo
  SerialBT.println("SISTEMA:🚪 Test servo: apertura/chiusura");
  gateServo.write(SERVO_OPEN_ANGLE);  // Apri
  delay(1000);
  gateServo.write(SERVO_CLOSED_ANGLE); // Chiudi
  delay(1000);
 
  // Statistiche comunicazione Bluetooth
  SerialBT.println("SISTEMA:📱 Bluetooth: " + String(messageCount) + " messaggi");
  delay(1000);
 
  // Risultato finale test
  if (sensorHealthy) {
    SerialBT.println("SISTEMA:✅ TUTTI I TEST OK!");
  } else {
    SerialBT.println("SISTEMA:⚠️ TEST COMPLETATO CON ERRORI SENSORE!");
  }
}

void showUsersViaBluetooth() {
  SerialBT.println("SISTEMA:👥 UTENTI NEL DATABASE:");
 
  if (sdCardAvailable) {
    // Lettura utenti da file CSV su SD
    File file = SD_MMC.open("/database.csv");
    if (file) {
      int count = 0;
      while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
       
        int separatorPos = line.indexOf(','); // Trova separatore username,password
        if (separatorPos > 0) {
          String username = line.substring(0, separatorPos);
          SerialBT.println("SISTEMA:- " + username + " (SD)");
          count++;
        }
      }
      file.close();
      SerialBT.println("SISTEMA:Totale: " + String(count) + " utenti da SD");
    } else {
      SerialBT.println("SISTEMA:❌ Errore lettura database.csv!");
      SerialBT.println("SISTEMA:Controllare presenza file database.csv");
    }
  } else {
    // Mostra utenti hardcoded in memoria
    for (int i = 0; i < NUM_MEMORY_USERS; i++) {
      SerialBT.println("SISTEMA:- " + memoryUsers[i].username + " (memoria)");
    }
    SerialBT.println("SISTEMA:Totale: " + String(NUM_MEMORY_USERS) + " utenti in memoria");
    SerialBT.println("SISTEMA:⚠️  Inserire scheda SD per database persistente");
  }
}

void showStats() {
  // Statistiche generali sistema
  unsigned long uptime = millis() / 1000;
  SerialBT.println("SISTEMA:📊 STATISTICHE:");
  SerialBT.println("SISTEMA:⏱️  Uptime: " + String(uptime) + "s");
  SerialBT.println("SISTEMA:📨 Messaggi BT: " + String(messageCount));
  SerialBT.println("SISTEMA:🔗 Bluetooth: CONNESSO");
  SerialBT.println("SISTEMA:💾 RAM libera: " + String(ESP.getFreeHeap()) + " bytes");
  SerialBT.println("SISTEMA:📏 Soglia sensore: " + String(DISTANCE_THRESHOLD) + "cm");
  SerialBT.println("SISTEMA:⏲️  Timer sbarra: " + String(GATE_DELAY_CLOSE/1000) + "s");
  SerialBT.println("SISTEMA:💳 Database: " + String(sdCardAvailable ? "SD CARD" : "MEMORIA"));
  SerialBT.println("SISTEMA:📡 Sensore: " + String(sensorHealthy ? "OK" : "ERRORE"));
 
  // Statistiche qualità sensore se disponibili
  if (sensorStats.totalMeasurements > 0) {
    float errorRate = (float)sensorStats.errorCount / sensorStats.totalMeasurements * 100;
    SerialBT.println("SISTEMA:📊 Errori sensore: " + String(errorRate, 1) + "%");
  }
}

void showSystemStatus() {
  // Array nomi stati per debug leggibile
  String stateNames[] = {"IDLE", "VEICOLO_RILEVATO", "USERNAME", "PASSWORD",
                        "ACCESSO_OK", "ACCESSO_NEGATO", "SBARRA_APERTA", "ATTESA_CHIUSURA", "ERRORE_SENSORE"};
 
  SerialBT.println("SISTEMA:🔍 STATO SISTEMA:");
  SerialBT.println("SISTEMA:📍 Stato: " + stateNames[currentSystemState]);
  SerialBT.println("SISTEMA:🚗 Veicolo presente: " + String(vehiclePresent ? "SI" : "NO"));
  SerialBT.println("SISTEMA:🚪 Sbarra: " + String(gateIsOpen ? "APERTA" : "CHIUSA"));
  SerialBT.println("SISTEMA:🔐 Autenticazione: " + String(authenticationInProgress ? "IN_CORSO" : "NO"));
  SerialBT.println("SISTEMA:📡 Sensore: " + String(sensorHealthy ? "FUNZIONANTE" : "ERRORE"));
 
  if (currentUser.length() > 0) {
    SerialBT.println("SISTEMA:👤 Utente corrente: " + currentUser);
  }
 
  // Warning se sensore in errore
  if (!sensorHealthy) {
    SerialBT.println("SISTEMA:⚠️ ATTENZIONE: Sensore non funzionante!");
    SerialBT.println("SISTEMA:Utilizzare 'sensor-test' per diagnostica");
  }
}

void testLogin(String loginCommand) {
  // Parsing comando formato: login:username,password
  int colonPos = loginCommand.indexOf(':');
  if (colonPos == -1) {
    SerialBT.println("SISTEMA:Formato errato. Usa: login:username,password");
    return;
  }
 
  String credentials = loginCommand.substring(colonPos + 1);
  int commaPos = credentials.indexOf(',');
 
  if (commaPos == -1) {
    SerialBT.println("SISTEMA:Formato errato. Usa: login:username,password");
    return;
  }
 
  String username = credentials.substring(0, commaPos);
  String password = credentials.substring(commaPos + 1);
 
  SerialBT.println("SISTEMA:🔐 Test login: " + username);
 
  // Test credenziali senza attivare sbarra
  if (validateUserCredentials(username, password)) {
    SerialBT.println("SISTEMA:✅ LOGIN RIUSCITO!");
    SerialBT.println("SISTEMA:Credenziali valide per: " + username);
  } else {
    SerialBT.println("SISTEMA:❌ LOGIN FALLITO!");
    SerialBT.println("SISTEMA:Credenziali errate per: " + username);
  }
}

int countUsers() {
  // Conta utenti in database attivo (SD o memoria)
  if (sdCardAvailable) {
    File file = SD_MMC.open("/database.csv");
    int count = 0;
    if (file) {
      while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.indexOf(',') > 0) count++; // Conta righe valide con separatore
      }
      file.close();
    }
    return count;
  } else {
    return NUM_MEMORY_USERS; // Utenti hardcoded
  }
}

// ===== FUNZIONI AUTENTICAZIONE =====
void handleUsernameValidation(String username) {
  currentUser = username; // Salva per fase successiva
 
  if (validateUsername(username)) {
    // Username trovato - procedi con password
    currentSystemState = REQUEST_PASSWORD;
    SerialBT.println("ACCESSO:Username '" + username + "' trovato!");
    SerialBT.println("ACCESSO:Inserisci la PASSWORD:");
    Serial.println("✓ Username valido: " + username);
  } else {
    // Username non trovato - incrementa tentativi
    authAttempts++;
    SerialBT.println("ACCESSO:Username '" + username + "' NON trovato!");
   
    if (authAttempts >= MAX_AUTH_ATTEMPTS) {
      // Troppi tentativi - blocca accesso
      SerialBT.println("ACCESSO:Troppi tentativi falliti. Accesso NEGATO!");
      Serial.println("✗ Troppi tentativi username errati");
      currentSystemState = ACCESS_DENIED;
    } else {
      // Permetti nuovo tentativo
      SerialBT.println("ACCESSO:Riprova (tentativo " + String(authAttempts + 1) + "/" + String(MAX_AUTH_ATTEMPTS) + "):");
    }
  }
}

void handlePasswordValidation(String password) {
  if (validateUserCredentials(currentUser, password)) {
    // Password corretta - autorizza accesso
    SerialBT.println("ACCESSO:PASSWORD CORRETTA!");
    SerialBT.println("ACCESSO:Benvenuto " + currentUser + "! Apertura in corso...");
    Serial.println("✓ Accesso autorizzato per: " + currentUser);
   
    logAccess(currentUser, true); // Log accesso riuscito
    currentSystemState = ACCESS_GRANTED;
  } else {
    // Password errata
    authAttempts++;
    SerialBT.println("ACCESSO:PASSWORD ERRATA!");
   
    if (authAttempts >= MAX_AUTH_ATTEMPTS) {
      // Troppi tentativi - nega accesso
      SerialBT.println("ACCESSO:Troppi tentativi falliti. Accesso NEGATO!");
      Serial.println("✗ Troppi tentativi password errati per: " + currentUser);
      logAccess(currentUser, false); // Log accesso negato
      currentSystemState = ACCESS_DENIED;
    } else {
      // Permetti nuovo tentativo password
      SerialBT.println("ACCESSO:Riprova password (tentativo " + String(authAttempts + 1) + "/" + String(MAX_AUTH_ATTEMPTS) + "):");
    }
  }
}

// ===== GESTIONE DATABASE =====
void initializeDatabase(bool sdAvailable) {
  sdCardAvailable = sdAvailable;
 
  if (sdCardAvailable) {
    // Controllo esistenza file database
    if (!SD_MMC.exists("/database.csv")) {
      Serial.println("📄 File database.csv non trovato, creazione...");
      createInitialDatabase(); // Crea database con utenti default
    } else {
      Serial.println("✓ Database database.csv trovato");
    }
   
    displayAvailableUsers(); // Mostra utenti per debug
  } else {
    // Modalità fallback memoria RAM
    Serial.println("💾 Utilizzando database in memoria RAM");
    Serial.println("👥 Utenti disponibili in memoria:");
    for (int i = 0; i < NUM_MEMORY_USERS; i++) {
      Serial.println("   - " + memoryUsers[i].username);
    }
    Serial.println("   Totale: " + String(NUM_MEMORY_USERS) + " utenti");
    Serial.println("⚠️  ATTENZIONE: Database non persistente!");
  }
}

void createInitialDatabase() {
  // Creazione file CSV con utenti predefiniti
  File file = SD_MMC.open("/database.csv", FILE_WRITE);
  if (file) {
    // Scrittura utenti in formato CSV (username,password)
    file.println("nico,nico");
    file.println("leo,leo");
    file.println("luca,luca");
    file.println("pietro,pietro");
    file.println("spezzino,dimerda");
    file.close();
    Serial.println("✓ Database database.csv creato con utenti reali");
  } else {
    Serial.println("✗ ERRORE: Impossibile creare database.csv!");
  }
}

void displayAvailableUsers() {
  if (sdCardAvailable) {
    Serial.println("👥 Utenti disponibili nel sistema (da database.csv):");
    File file = SD_MMC.open("/database.csv");
    if (file) {
      int userCount = 0;
      while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
       
        int separatorPos = line.indexOf(',');
        if (separatorPos > 0) {
          String username = line.substring(0, separatorPos);
          Serial.println("   - " + username);
          userCount++;
        }
      }
      file.close();
      Serial.println("   Totale: " + String(userCount) + " utenti");
    } else {
      Serial.println("⚠️ Impossibile leggere database.csv");
    }
  }
  // Se SD non disponibile, utenti già mostrati in initializeDatabase()
}

bool validateUsername(String username) {
  if (sdCardAvailable) {
    // Validazione da file CSV su SD
    File file = SD_MMC.open("/database.csv");
    if (!file) {
      Serial.println("✗ Errore lettura database.csv!");
      return false;
    }
   
    // Scansione file per trovare username
    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();
     
      int separatorPos = line.indexOf(',');
      if (separatorPos > 0) {
        String fileUsername = line.substring(0, separatorPos);
        if (fileUsername.equals(username)) {
          file.close();
          return true; // Username trovato
        }
      }
    }
   
    file.close();
    return false; // Username non trovato
  } else {
    // Validazione da array in memoria
    for (int i = 0; i < NUM_MEMORY_USERS; i++) {
      if (memoryUsers[i].username.equals(username)) {
        return true;
      }
    }
    return false;
  }
}

bool validateUserCredentials(String username, String password) {
  if (sdCardAvailable) {
    // Validazione completa da file CSV
    File file = SD_MMC.open("/database.csv");
    if (!file) {
      Serial.println("✗ Errore lettura database.csv!");
      return false;
    }
   
    // Scansione per trovare coppia username/password
    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();
     
      int separatorPos = line.indexOf(',');
      if (separatorPos > 0) {
        String fileUsername = line.substring(0, separatorPos);
        String filePassword = line.substring(separatorPos + 1);
       
        // Match esatto di entrambi i campi
        if (fileUsername.equals(username) && filePassword.equals(password)) {
          file.close();
          return true; // Credenziali valide
        }
      }
    }
   
    file.close();
    return false; // Credenziali non valide
  } else {
    // Validazione da array in memoria
    for (int i = 0; i < NUM_MEMORY_USERS; i++) {
      if (memoryUsers[i].username.equals(username) &&
          memoryUsers[i].password.equals(password)) {
        return true;
      }
    }
    return false;
  }
}

// ===== CONTROLLO SBARRA =====
void openGate() {
  Serial.println("🚪 Apertura sbarra...");
  gateServo.write(SERVO_OPEN_ANGLE); // Movimento servo ad angolo apertura
  gateIsOpen = true;
  gateOpenedTime = millis(); // Timestamp per timer sicurezza
}

void closeGate() {
  Serial.println("🚪 Chiusura sbarra...");
  gateServo.write(SERVO_CLOSED_ANGLE); // Ritorno servo a posizione chiusa
  gateIsOpen = false;
  resetToIdle(); // Reset completo sistema
}

// ===== GESTIONE TIMEOUT =====
void checkTimeouts() {
  unsigned long currentTime = millis();
 
  // Timeout autenticazione utente
  if (authenticationInProgress &&
      (currentTime - authStartTime > AUTH_TIMEOUT)) {
    Serial.println("⏰ Timeout autenticazione");
    SerialBT.println("ACCESSO:Timeout! Riprova avvicinando il veicolo.");
    resetToIdle(); // Reset per timeout
  }
 
  // Timer sicurezza sbarra (fallback se altri timer falliscono)
  if (gateIsOpen && currentSystemState == GATE_OPERATING &&
      (currentTime - gateOpenedTime > GATE_OPEN_DURATION)) {
    Serial.println("⏰ Timer sicurezza sbarra - chiusura forzata");
    closeGate(); // Chiusura forzata per sicurezza
  }
}

// ===== RESET SISTEMA =====
void resetToIdle() {
  // Reset completo stato sistema a condizione iniziale
  authenticationInProgress = false;
  currentSystemState = IDLE_STATE;
  currentUser = "";
  authAttempts = 0;
  // NON resettare vehicleLeftTime qui - serve per il timer di chiusura
 
  Serial.println("🔄 Sistema reset - Pronto per nuovo accesso");
}
