#include "BluetoothSerial.h"
#include "FS.h"
#include "SD_MMC.h"  // Cambiato da SD.h a SD_MMC.h
#include "SPI.h"
#include <ESP32Servo.h>

// ===== CONFIGURAZIONE PIN =====
#define TRIG_PIN 0        // Pin trigger sensore ultrasuoni (ripristinato dal primo file)
#define ECHO_PIN 33       // Pin echo sensore ultrasuoni (ripristinato dal primo file)
#define SERVO_PIN 13      // Pin controllo servo motore

// ===== CONFIGURAZIONE SD_MMC (dal primo file) =====
#define SD_MMC_CMD 15     // Non cambiare
#define SD_MMC_CLK 14     // Non cambiare
#define SD_MMC_D0 2       // Non cambiare

// ===== CONFIGURAZIONE SENSORE =====
#define DISTANCE_THRESHOLD 20    // cm - distanza massima per rilevare veicolo (modificato)
#define MIN_DETECTION_TIME 3000  // ms - tempo minimo presenza per attivare sistema
#define MAX_DISTANCE 400         // cm - distanza massima sensore (filtro errori)

// ===== CONFIGURAZIONE DIAGNOSTICA SENSORE =====
#define SENSOR_TEST_SAMPLES 10      // numero campioni per test diagnostico
#define SENSOR_ERROR_THRESHOLD 5    // errori consecutivi per segnalare malfunzionamento
#define SENSOR_TIMEOUT_US 30000     // timeout lettura sensore in microsecondi
#define SENSOR_MIN_DISTANCE 2       // cm - distanza minima valida
#define SENSOR_STABILITY_SAMPLES 5  // campioni per test stabilità
#define SENSOR_VARIANCE_THRESHOLD 10 // cm - varianza massima accettabile per stabilità

// ===== CONFIGURAZIONE SERVO =====
#define SERVO_CLOSED_ANGLE 0     // gradi - angolo sbarra chiusa
#define SERVO_OPEN_ANGLE 90      // gradi - angolo sbarra aperta  
#define GATE_OPEN_DURATION 5000  // ms - tempo apertura sbarra (5 secondi - modificato)
#define GATE_DELAY_CLOSE 5000    // ms - ritardo chiusura dopo che sensore non rileva più (nuovo)

// ===== CONFIGURAZIONE SICUREZZA =====
#define MAX_AUTH_ATTEMPTS 3      // tentativi massimi per sessione
#define AUTH_TIMEOUT 10000       // ms - timeout autenticazione (10 sec)

BluetoothSerial SerialBT;
Servo gateServo;

// ===== VARIABILI GLOBALI =====
bool vehiclePresent = false;
bool authenticationInProgress = false;
bool gateIsOpen = false;
unsigned long vehicleDetectedTime = 0;
unsigned long vehicleLeftTime = 0;    // nuovo - quando il veicolo se ne va
unsigned long gateOpenedTime = 0;
unsigned long authStartTime = 0;
String currentUser = "";
int authAttempts = 0;
int messageCount = 0;  // nuovo - per statistiche bluetooth
bool sdCardAvailable = false;  // nuovo - stato scheda SD

// ===== VARIABILI DIAGNOSTICA SENSORE =====
bool sensorHealthy = true;
int consecutiveErrors = 0;
unsigned long totalReadings = 0;
unsigned long errorReadings = 0;
unsigned long lastSensorTest = 0;
float lastValidDistance = -1;
bool sensorCalibrated = false;

// ===== STATISTICHE SENSORE =====
struct SensorStats {
  unsigned long totalMeasurements;
  unsigned long errorCount;
  unsigned long timeoutCount;
  float minDistance;
  float maxDistance;
  float avgDistance;
  unsigned long lastResetTime;
};
SensorStats sensorStats = {0, 0, 0, 999.0, 0.0, 0.0, 0};

// ===== DATABASE IN MEMORIA (fallback senza SD) =====
struct MemoryUser {
  String username;
  String password;
};

MemoryUser memoryUsers[] = {
  {"nico", "nico"},
  {"leo", "leo"},
  {"luca", "luca"},
  {"pietro", "pietro"},
  {"spezzino", "dimerda"}
};

const int NUM_MEMORY_USERS = sizeof(memoryUsers) / sizeof(memoryUsers[0]);

// ===== STATI DEL SISTEMA =====
enum SystemState {
  IDLE_STATE,
  VEHICLE_DETECTED,
  REQUEST_USERNAME,
  REQUEST_PASSWORD,
  ACCESS_GRANTED,
  ACCESS_DENIED,
  GATE_OPERATING,
  GATE_WAITING_CLOSE,  // nuovo - attesa prima della chiusura
  SENSOR_ERROR         // nuovo - stato errore sensore
};

SystemState currentSystemState = IDLE_STATE;

void setup() {
  Serial.begin(115200);
  delay(1000);
 
  Serial.println("=================================");
  Serial.println("  SISTEMA CONTROLLO ACCESSI");
  Serial.println("  Progetto Scolastico ESP32");
  Serial.println("  Versione con Diagnostica Sensore");
  Serial.println("=================================");
 
  // ===== INIZIALIZZAZIONE BLUETOOTH =====
  SerialBT.begin("ESP32_Controllo_Accessi");
  Serial.println("✓ Bluetooth attivo: ESP32_Controllo_Accessi");
 
  // ===== INIZIALIZZAZIONE SENSORE ULTRASUONI =====
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
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
  gateServo.write(SERVO_CLOSED_ANGLE);
  Serial.println("✓ Servo sbarra posizionato (chiuso)");
 
  // ===== INIZIALIZZAZIONE MICROSD (configurazione dal primo file) =====
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if(!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("⚠️ ATTENZIONE: MicroSD non rilevata!");
    Serial.println("  Sistema funzionerà con database in memoria");
    Serial.println("  Le credenziali NON saranno salvate permanentemente");
    sdCardAvailable = false;
  } else {
    sdCardAvailable = true;
    Serial.println("✓ MicroSD inizializzata correttamente");
   
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
 
  // ===== VERIFICA/CREAZIONE DATABASE =====
  initializeDatabase(sdCardAvailable);
 
  // ===== INIZIALIZZAZIONE STATISTICHE =====
  sensorStats.lastResetTime = millis();
  sensorStats.minDistance = 999.0;
 
  Serial.println("=================================");
  Serial.println("SISTEMA PRONTO!");
  Serial.println("In attesa di veicoli...");
  Serial.println("Comandi Bluetooth disponibili:");
  Serial.println("- help, test, users, stats, ping");
  Serial.println("- sensor, calibrate, reset-sensor");
  Serial.println("=================================");
 
  // Messaggio di benvenuto via Bluetooth
  SerialBT.println("SISTEMA:Bluetooth connesso - Sistema Controllo Accessi");
  SerialBT.println("SISTEMA:Diagnostica sensore: " + String(sensorHealthy ? "OK" : "ERRORE"));
  SerialBT.println("SISTEMA:Digita 'help' per comandi di test");
}

void loop() {
  // Controlla presenza veicolo con sensore
  checkVehicleDetection();
 
  // Gestisce la macchina a stati
  handleSystemStates();
 
  // Processa messaggi Bluetooth (sia per autenticazione che per test)
  processBluetoothMessages();
 
  // Controlla timeout e timer
  checkTimeouts();
 
  // Diagnostica periodica sensore (ogni 30 secondi)
  if (millis() - lastSensorTest > 30000) {
    checkSensorHealth();
    lastSensorTest = millis();
  }
 
  delay(50); // Piccola pausa per ottimizzare performance
}

// ===== FUNZIONI SENSORE ULTRASUONI =====
void checkVehicleDetection() {
  long distance = measureDistance();
 
  // Aggiorna statistiche
  updateSensorStats(distance);
 
  // Filtra letture errate
  if (distance > 0 && distance < MAX_DISTANCE) {
   
    if (distance <= DISTANCE_THRESHOLD) {
      // Veicolo rilevato
      if (!vehiclePresent) {
        vehiclePresent = true;
        vehicleDetectedTime = millis();
        vehicleLeftTime = 0; // reset timer uscita
        Serial.println("📡 Veicolo rilevato a " + String(distance) + " cm");
       
        // Reset errori consecutivi se lettura valida
        consecutiveErrors = 0;
        if (!sensorHealthy && consecutiveErrors == 0) {
          Serial.println("✅ Sensore sembra essere tornato funzionante");
          sensorHealthy = true;
          if (currentSystemState == SENSOR_ERROR) {
            currentSystemState = IDLE_STATE;
          }
        }
      }
     
      // Controlla se veicolo presente abbastanza a lungo
      if (millis() - vehicleDetectedTime > MIN_DETECTION_TIME &&
          currentSystemState == IDLE_STATE && sensorHealthy) {
        startAuthenticationProcess();
      }
    } else {
      // Nessun veicolo
      if (vehiclePresent) {
        vehiclePresent = false;
        vehicleLeftTime = millis(); // segna quando il veicolo se ne va
        Serial.println("📡 Veicolo non più presente - avvio timer chiusura");
       
        // Se sbarra è aperta, inizia countdown per chiusura
        if (gateIsOpen && currentSystemState == GATE_OPERATING) {
          currentSystemState = GATE_WAITING_CLOSE;
        }
        // Se non in autenticazione, reset
        else if (!authenticationInProgress) {
          resetToIdle();
        }
      }
    }
  } else {
    // Lettura errata - incrementa contatore errori
    consecutiveErrors++;
    if (consecutiveErrors >= SENSOR_ERROR_THRESHOLD) {
      if (sensorHealthy) {
        Serial.println("❌ ERRORE SENSORE: Troppi errori consecutivi!");
        SerialBT.println("SISTEMA:⚠️ ERRORE SENSORE RILEVATO!");
        sensorHealthy = false;
        currentSystemState = SENSOR_ERROR;
      }
    }
  }
}

long measureDistance() {
  totalReadings++;
 
  // Impulso trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
 
  // Misura durata echo con timeout
  long duration = pulseIn(ECHO_PIN, HIGH, SENSOR_TIMEOUT_US);
 
  if (duration == 0) {
    errorReadings++;
    sensorStats.timeoutCount++;
    return -1; // Timeout
  }
 
  // Calcola distanza in cm
  long distance = duration * 0.034 / 2;
 
  // Verifica validità della lettura
  if (distance < SENSOR_MIN_DISTANCE || distance > MAX_DISTANCE) {
    errorReadings++;
    sensorStats.errorCount++;
    return -2; // Lettura non valida
  }
 
  lastValidDistance = distance;
  return distance;
}

// ===== FUNZIONI DIAGNOSTICA SENSORE =====
bool performSensorDiagnostic() {
  Serial.println("🔍 Esecuzione diagnostica completa sensore...");
  SerialBT.println("SISTEMA:🔍 Test sensore in corso...");
 
  int validReadings = 0;
  int timeouts = 0;
  int invalidReadings = 0;
  float readings[SENSOR_TEST_SAMPLES];
 
  // Esegui campionamento
  for (int i = 0; i < SENSOR_TEST_SAMPLES; i++) {
    long distance = measureDistance();
   
    if (distance == -1) {
      timeouts++;
    } else if (distance == -2) {
      invalidReadings++;
    } else {
      readings[validReadings] = distance;
      validReadings++;
    }
   
    delay(100); // Piccola pausa tra le letture
  }
 
  // Analisi risultati
  float successRate = (float)validReadings / SENSOR_TEST_SAMPLES * 100;
 
  Serial.println("📊 Risultati diagnostica:");
  Serial.println("   Letture valide: " + String(validReadings) + "/" + String(SENSOR_TEST_SAMPLES));
  Serial.println("   Timeout: " + String(timeouts));
  Serial.println("   Letture invalide: " + String(invalidReadings));
  Serial.println("   Tasso successo: " + String(successRate) + "%");
 
  SerialBT.println("SISTEMA:📊 Letture valide: " + String(validReadings) + "/" + String(SENSOR_TEST_SAMPLES));
  SerialBT.println("SISTEMA:📊 Tasso successo: " + String(successRate) + "%");
 
  // Test stabilità se abbiamo abbastanza letture valide
  if (validReadings >= SENSOR_STABILITY_SAMPLES) {
    float variance = calculateVariance(readings, validReadings);
    Serial.println("   Varianza: " + String(variance) + " cm");
    SerialBT.println("SISTEMA:📊 Stabilità: " + String(variance < SENSOR_VARIANCE_THRESHOLD ? "BUONA" : "INSTABILE"));
   
    if (variance > SENSOR_VARIANCE_THRESHOLD) {
      Serial.println("⚠️ Sensore instabile - varianza troppo alta");
      return false;
    }
  }
 
  // Considera il sensore OK se almeno 70% delle letture sono valide
  return successRate >= 70.0;
}

float calculateVariance(float readings[], int count) {
  if (count < 2) return 0;
 
  // Calcola media
  float sum = 0;
  for (int i = 0; i < count; i++) {
    sum += readings[i];
  }
  float mean = sum / count;
 
  // Calcola varianza
  float variance = 0;
  for (int i = 0; i < count; i++) {
    variance += pow(readings[i] - mean, 2);
  }
 
  return sqrt(variance / (count - 1));
}

void checkSensorHealth() {
  // Controlla tasso di errore generale
  if (totalReadings > 100) {
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
    // Aggiorna min/max
    if (distance < sensorStats.minDistance) {
      sensorStats.minDistance = distance;
    }
    if (distance > sensorStats.maxDistance) {
      sensorStats.maxDistance = distance;
    }
   
    // Aggiorna media mobile semplificata
    sensorStats.avgDistance = (sensorStats.avgDistance * 0.9) + (distance * 0.1);
  } else {
    sensorStats.errorCount++;
  }
}

void resetSensorStats() {
  sensorStats.totalMeasurements = 0;
  sensorStats.errorCount = 0;
  sensorStats.timeoutCount = 0;
  sensorStats.minDistance = 999.0;
  sensorStats.maxDistance = 0.0;
  sensorStats.avgDistance = 0.0;
  sensorStats.lastResetTime = millis();
 
  totalReadings = 0;
  errorReadings = 0;
  consecutiveErrors = 0;
 
  Serial.println("🔄 Statistiche sensore resettate");
  SerialBT.println("SISTEMA:🔄 Statistiche sensore resettate");
}

// ===== GESTIONE STATI SISTEMA =====
void handleSystemStates() {
  switch (currentSystemState) {
    case IDLE_STATE:
      // Sistema in attesa
      break;
     
    case VEHICLE_DETECTED:
      // Già gestito in checkVehicleDetection()
      break;
     
    case REQUEST_USERNAME:
    case REQUEST_PASSWORD:
      // Gestiti in processBluetoothMessages()
      break;
     
    case ACCESS_GRANTED:
      openGate();
      currentSystemState = GATE_OPERATING;
      break;
     
    case ACCESS_DENIED:
      delay(2000); // Pausa prima di reset
      resetToIdle();
      break;
     
    case GATE_OPERATING:
      // Timer gestito in checkTimeouts()
      break;
     
    case GATE_WAITING_CLOSE:
      // Aspetta 5 secondi dopo che il veicolo se ne va, poi chiude
      if (vehicleLeftTime > 0 && millis() - vehicleLeftTime > GATE_DELAY_CLOSE) {
        closeGate();
      }
      break;
     
    case SENSOR_ERROR:
      // Sistema bloccato per errore sensore - richiede intervento manuale
      break;
  }
}

// ===== PROCESSO AUTENTICAZIONE =====
void startAuthenticationProcess() {
  if (!sensorHealthy) {
    Serial.println("⚠️ Autenticazione bloccata - sensore non funzionante");
    SerialBT.println("SISTEMA:⚠️ Servizio temporaneamente non disponibile");
    return;
  }
 
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
    receivedMessage.trim();
   
    if (receivedMessage.length() > 0) {
      messageCount++;
      Serial.println("📱 [" + String(messageCount) + "] Ricevuto: '" + receivedMessage + "'");
     
      // Se il sistema è in modalità autenticazione, processa le credenziali
      if (currentSystemState == REQUEST_USERNAME || currentSystemState == REQUEST_PASSWORD) {
        switch (currentSystemState) {
          case REQUEST_USERNAME:
            handleUsernameValidation(receivedMessage);
            break;
           
          case REQUEST_PASSWORD:
            handlePasswordValidation(receivedMessage);
            break;
        }
      }
      // Altrimenti processa i comandi di test
      else {
        processTestCommand(receivedMessage);
      }
    }
  }
}

// ===== NUOVE FUNZIONI BLUETOOTH TEST =====
void processTestCommand(String command) {
  command.toLowerCase();
 
  if (command == "help" || command == "aiuto") {
    showHelp();
  }
  else if (command == "test") {
    runSystemTest();
  }
  else if (command == "users" || command == "utenti") {
    showUsersViaBluetooth();
  }
  else if (command == "stats" || command == "statistiche") {
    showStats();
  }
  else if (command == "ping") {
    SerialBT.println("SISTEMA:PONG! Sistema attivo");
  }
  else if (command == "status" || command == "stato") {
    showSystemStatus();
  }
  else if (command == "sensor" || command == "sensore") {
    showSensorDiagnostics();
  }
  else if (command == "calibrate" || command == "calibra") {
    calibrateSensor();
  }
  else if (command == "reset-sensor" || command == "reset-sensore") {
    resetSensorStats();
  }
  else if (command == "sensor-test" || command == "test-sensore") {
    runSensorTest();
  }
  else if (command.startsWith("login:")) {
    testLogin(command);
  }
  else if (command == "open" && currentSystemState == IDLE_STATE) {
    // Comando debug per aprire manualmente la sbarra
    SerialBT.println("SISTEMA:🔧 Apertura manuale sbarra (debug)");
    openGate();
    currentSystemState = GATE_OPERATING;
  }
  else if (command == "close" && gateIsOpen) {
    // Comando debug per chiudere manualmente la sbarra
    SerialBT.println("SISTEMA:🔧 Chiusura manuale sbarra (debug)");
    closeGate();
  }
  else if (command == "force-reset" && currentSystemState == SENSOR_ERROR) {
    // Reset forzato per uscire dallo stato errore
    SerialBT.println("SISTEMA:🔧 Reset forzato stato errore sensore");
    sensorHealthy = true;
    consecutiveErrors = 0;
    resetToIdle();
  }
  else {
    // Se il sistema non è in autenticazione, informa dei comandi disponibili
    if (currentSystemState == IDLE_STATE || currentSystemState == SENSOR_ERROR) {
      SerialBT.println("SISTEMA:Comando '" + command + "' non riconosciuto");
      SerialBT.println("SISTEMA:Digita 'help' per vedere i comandi disponibili");
    } else {
      SerialBT.println("ACCESSO:Sistema non in modalità test");
    }
  }
}

void showHelp() {
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
  SerialBT.println("SISTEMA:🔍 === DIAGNOSTICA SENSORE ===");
  SerialBT.println("SISTEMA:📊 Stato: " + String(sensorHealthy ? "FUNZIONANTE" : "ERRORE"));
  SerialBT.println("SISTEMA:📊 Letture totali: " + String(sensorStats.totalMeasurements));
  SerialBT.println("SISTEMA:📊 Errori: " + String(sensorStats.errorCount));
  SerialBT.println("SISTEMA:📊 Timeout: " + String(sensorStats.timeoutCount));
 
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
 
  unsigned long uptime = (millis() - sensorStats.lastResetTime) / 1000;
  SerialBT.println("SISTEMA:📊 Uptime sensore: " + String(uptime) + "s");
  SerialBT.println("SISTEMA:================================");
}

void calibrateSensor() {
  SerialBT.println("SISTEMA:🎯 Avvio calibrazione sensore...");
  SerialBT.println("SISTEMA:Assicurati che davanti al sensore sia libero");
  delay(3000);
 
  // Esegui calibrazione con ambiente libero
  float calibrationReadings[10];
  int validCalibrations = 0;
 
  for (int i = 0; i < 10; i++) {
    long distance = measureDistance();
    if (distance > 0 && distance < MAX_DISTANCE) {
      calibrationReadings[validCalibrations] = distance;
      validCalibrations++;
    }
    delay(200);
    SerialBT.println("SISTEMA:Campione " + String(i+1) + "/10...");
  }
 
  if (validCalibrations >= 7) {
    // Calcola distanza ambiente libero
    float sum = 0;
    for (int i = 0; i < validCalibrations; i++) {
      sum += calibrationReadings[i];
    }
    float avgDistance = sum / validCalibrations;
   
    SerialBT.println("SISTEMA:✅ Calibrazione completata!");
    SerialBT.println("SISTEMA:📏 Distanza ambiente libero: " + String(avgDistance, 1) + " cm");
    SerialBT.println("SISTEMA:📏 Soglia rilevamento: " + String(DISTANCE_THRESHOLD) + " cm");
   
    sensorCalibrated = true;
    resetSensorStats(); // Reset statistiche dopo calibrazione
  } else {
    SerialBT.println("SISTEMA:❌ Calibrazione fallita!");
    SerialBT.println("SISTEMA:Troppi errori di lettura sensore");
  }
}

void runSensorTest() {
  SerialBT.println("SISTEMA:🧪 TEST SENSORE COMPLETO...");
 
  // Reset temporaneo contatori per test pulito
  int oldConsecutiveErrors = consecutiveErrors;
  consecutiveErrors = 0;
 
  bool testResult = performSensorDiagnostic();
 
  if (testResult) {
    SerialBT.println("SISTEMA:✅ SENSORE: FUNZIONANTE");
    sensorHealthy = true;
    if (currentSystemState == SENSOR_ERROR) {
      currentSystemState = IDLE_STATE;
      SerialBT.println("SISTEMA:🔄 Sistema ripristinato da errore sensore");
    }
  } else {
    SerialBT.println("SISTEMA:❌ SENSORE: PROBLEMI RILEVATI");
    SerialBT.println("SISTEMA:Controllare collegamenti e ostacoli");
    sensorHealthy = false;
  }
 
  // Ripristina contatore se era zero prima del test
  if (oldConsecutiveErrors == 0 && !testResult) {
    consecutiveErrors = oldConsecutiveErrors;
  }
}

void runSystemTest() {
  SerialBT.println("SISTEMA:🧪 TEST SISTEMA COMPLETO...");
  delay(500);
 
  // Test sensore
  SerialBT.println("SISTEMA:📡 Test sensore ultrasuoni...");
  long dist = measureDistance();
  SerialBT.println("SISTEMA:📡 Distanza attuale: " + String(dist) + "cm");
  SerialBT.println("SISTEMA:📡 Stato sensore: " + String(sensorHealthy ? "OK" : "ERRORE"));
  delay(1000);
 
  SerialBT.println("SISTEMA:💾 Database: " + String(countUsers()) + " utenti (" +
                   String(sdCardAvailable ? "SD" : "memoria") + ")");
  delay(1000);
 
  SerialBT.println("SISTEMA:🚪 Test servo: apertura/chiusura");
  gateServo.write(SERVO_OPEN_ANGLE);
  delay(1000);
  gateServo.write(SERVO_CLOSED_ANGLE);
  delay(1000);
 
  SerialBT.println("SISTEMA:📱 Bluetooth: " + String(messageCount) + " messaggi");
  delay(1000);
 
  if (sensorHealthy) {
    SerialBT.println("SISTEMA:✅ TUTTI I TEST OK!");
  } else {
    SerialBT.println("SISTEMA:⚠️ TEST COMPLETATO CON ERRORI SENSORE!");
  }
}

void showUsersViaBluetooth() {
  SerialBT.println("SISTEMA:👥 UTENTI NEL DATABASE:");
 
  if (sdCardAvailable) {
    // Leggi da database.csv (nome file dal primo file)
    File file = SD_MMC.open("/database.csv");
    if (file) {
      int count = 0;
      while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
       
        int separatorPos = line.indexOf(',');
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
    // Mostra utenti in memoria
    for (int i = 0; i < NUM_MEMORY_USERS; i++) {
      SerialBT.println("SISTEMA:- " + memoryUsers[i].username + " (memoria)");
    }
    SerialBT.println("SISTEMA:Totale: " + String(NUM_MEMORY_USERS) + " utenti in memoria");
    SerialBT.println("SISTEMA:⚠️  Inserire scheda SD per database persistente");
  }
}

void showStats() {
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
 
  // Statistiche sensore
  if (sensorStats.totalMeasurements > 0) {
    float errorRate = (float)sensorStats.errorCount / sensorStats.totalMeasurements * 100;
    SerialBT.println("SISTEMA:📊 Errori sensore: " + String(errorRate, 1) + "%");
  }
}

void showSystemStatus() {
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
 
  if (!sensorHealthy) {
    SerialBT.println("SISTEMA:⚠️ ATTENZIONE: Sensore non funzionante!");
    SerialBT.println("SISTEMA:Utilizzare 'sensor-test' per diagnostica");
  }
}

void testLogin(String loginCommand) {
  // Formato: login:username,password
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
 
  if (validateUserCredentials(username, password)) {
    SerialBT.println("SISTEMA:✅ LOGIN RIUSCITO!");
    SerialBT.println("SISTEMA:Credenziali valide per: " + username);
  } else {
    SerialBT.println("SISTEMA:❌ LOGIN FALLITO!");
    SerialBT.println("SISTEMA:Credenziali errate per: " + username);
  }
}

int countUsers() {
  if (sdCardAvailable) {
    File file = SD_MMC.open("/database.csv");
    int count = 0;
    if (file) {
      while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.indexOf(',') > 0) count++;
      }
      file.close();
    }
    return count;
  } else {
    return NUM_MEMORY_USERS;
  }
}

// ===== FUNZIONI AUTENTICAZIONE =====
void handleUsernameValidation(String username) {
  currentUser = username;
 
  if (validateUsername(username)) {
    currentSystemState = REQUEST_PASSWORD;
    SerialBT.println("ACCESSO:Username '" + username + "' trovato!");
    SerialBT.println("ACCESSO:Inserisci la PASSWORD:");
    Serial.println("✓ Username valido: " + username);
  } else {
    authAttempts++;
    SerialBT.println("ACCESSO:Username '" + username + "' NON trovato!");
   
    if (authAttempts >= MAX_AUTH_ATTEMPTS) {
      SerialBT.println("ACCESSO:Troppi tentativi falliti. Accesso NEGATO!");
      Serial.println("✗ Troppi tentativi username errati");
      currentSystemState = ACCESS_DENIED;
    } else {
      SerialBT.println("ACCESSO:Riprova (tentativo " + String(authAttempts + 1) + "/" + String(MAX_AUTH_ATTEMPTS) + "):");
    }
  }
}

void handlePasswordValidation(String password) {
  if (validateUserCredentials(currentUser, password)) {
    SerialBT.println("ACCESSO:PASSWORD CORRETTA!");
    SerialBT.println("ACCESSO:Benvenuto " + currentUser + "! Apertura in corso...");
    Serial.println("✓ Accesso autorizzato per: " + currentUser);
   
    logAccess(currentUser, true);
    currentSystemState = ACCESS_GRANTED;
  } else {
    authAttempts++;
    SerialBT.println("ACCESSO:PASSWORD ERRATA!");
   
    if (authAttempts >= MAX_AUTH_ATTEMPTS) {
      SerialBT.println("ACCESSO:Troppi tentativi falliti. Accesso NEGATO!");
      Serial.println("✗ Troppi tentativi password errati per: " + currentUser);
      logAccess(currentUser, false);
      currentSystemState = ACCESS_DENIED;
    } else {
      SerialBT.println("ACCESSO:Riprova password (tentativo " + String(authAttempts + 1) + "/" + String(MAX_AUTH_ATTEMPTS) + "):");
    }
  }
}

// ===== GESTIONE DATABASE =====
void initializeDatabase(bool sdAvailable) {
  sdCardAvailable = sdAvailable;
 
  if (sdCardAvailable) {
    if (!SD_MMC.exists("/database.csv")) {
      Serial.println("📄 File database.csv non trovato, creazione...");
      createInitialDatabase();
    } else {
      Serial.println("✓ Database database.csv trovato");
    }
   
    // Mostra utenti disponibili (per debug)
    displayAvailableUsers();
  } else {
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
  File file = SD_MMC.open("/database.csv", FILE_WRITE);
  if (file) {
    // Utenti dal database reale
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
  // Se SD non disponibile, gli utenti sono già mostrati in initializeDatabase()
}

bool validateUsername(String username) {
  if (sdCardAvailable) {
    // Validazione da database.csv
    File file = SD_MMC.open("/database.csv");
    if (!file) {
      Serial.println("✗ Errore lettura database.csv!");
      return false;
    }
   
    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();
     
      int separatorPos = line.indexOf(',');
      if (separatorPos > 0) {
        String fileUsername = line.substring(0, separatorPos);
        if (fileUsername.equals(username)) {
          file.close();
          return true;
        }
      }
    }
   
    file.close();
    return false;
  } else {
    // Validazione da memoria
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
    // Validazione da database.csv
    File file = SD_MMC.open("/database.csv");
    if (!file) {
      Serial.println("✗ Errore lettura database.csv!");
      return false;
    }
   
    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();
     
      int separatorPos = line.indexOf(',');
      if (separatorPos > 0) {
        String fileUsername = line.substring(0, separatorPos);
        String filePassword = line.substring(separatorPos + 1);
       
        if (fileUsername.equals(username) && filePassword.equals(password)) {
          file.close();
          return true;
        }
      }
    }
   
    file.close();
    return false;
  } else {
    // Validazione da memoria
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
  gateServo.write(SERVO_OPEN_ANGLE);
  gateIsOpen = true;
  gateOpenedTime = millis();
}


void closeGate() {
  Serial.println("🚪 Chiusura sbarra...");
  gateServo.write(SERVO_CLOSED_ANGLE);
  gateIsOpen = false;
  resetToIdle();
}

// ===== GESTIONE TIMEOUT =====
void checkTimeouts() {
  unsigned long currentTime = millis();
 
  // Timeout autenticazione
  if (authenticationInProgress &&
      (currentTime - authStartTime > AUTH_TIMEOUT)) {
    Serial.println("⏰ Timeout autenticazione");
    SerialBT.println("ACCESSO:Timeout! Riprova avvicinando il veicolo.");
    resetToIdle();
  }
 
  // Timer chiusura sbarra (fallback se non gestito da GATE_WAITING_CLOSE)
  if (gateIsOpen && currentSystemState == GATE_OPERATING &&
      (currentTime - gateOpenedTime > GATE_OPEN_DURATION)) {
    Serial.println("⏰ Timer sicurezza sbarra - chiusura forzata");
    closeGate();
  }
}

// ===== LOG ACCESSI =====
void logAccess(String username, bool success) {
  String timestamp = String(millis() / 1000); // timestamp semplificato
  String status = success ? "AUTORIZZATO" : "NEGATO";
 
  Serial.println("📝 Log: " + username + " - " + status);
 
  if (sdCardAvailable) {
    // Salva su SD se disponibile
    File logFile = SD_MMC.open("/log_accessi.txt", FILE_APPEND);
    if (logFile) {
      logFile.println(timestamp + "," + username + "," + status);
      logFile.close();
      Serial.println("📝 Log salvato su SD");
    } else {
      Serial.println("⚠️ Impossibile salvare log su SD");
    }
  } else {
    Serial.println("⚠️ Log non salvato - SD non disponibile");
  }
}

// ===== RESET SISTEMA =====
void resetToIdle() {
  authenticationInProgress = false;
  currentSystemState = IDLE_STATE;
  currentUser = "";
  authAttempts = 0;
  vehicleLeftTime = 0;
 
  Serial.println("🔄 Sistema reset - Pronto per nuovo accesso");
}