#  Sbarra Automatica con ESP32 e Autenticazione via Bluetooth

Progetto didattico realizzato con ESP32 per creare una **sbarra automatica** che si apre **solo se l'utente inserisce credenziali corrette** via Bluetooth. </br>
Il sistema rileva la presenza di un veicolo, chiede l'autenticazione, controlla le credenziali da una **scheda SD**, e controlla un **servomotore** per aprire o chiudere la sbarra.

</br>

### 📑 Indice

- [⚙️ Funzionalità principali](#️-funzionalità-principali)
- [🧩 Componenti utilizzati](-Componenti-utilizzati)
- [🔄 Funzionamento generale](#funzionamento-generale)
  - [🧪 Diagnostica iniziale](#-diagnostica-iniziale)
  - [🚗 Rilevamento veicolo](#-rilevamento-veicolo)
  - [🔐 Autenticazione via Bluetooth](#-autenticazione-via-bluetooth)
  - [📤 Verifica e controllo della sbarra](#-verifica-e-controllo-della-sbarra)
- [🗂️ Database utenti](#️-database-utenti)
- [📲 Interfaccia utente](#-interfaccia-utente)
- [💬 Comandi via Bluetooth](#-comandi-per-diagnostica-via-bluetooth)
- [🧠 Requisiti software](#-requisiti-software)

</br>

---

</br>

## ⚙️ Funzionalità principali

- ✅ Diagnostica completa all'avvio
- 🚗 Rilevamento del veicolo tramite sensore a ultrasuoni
- 📲 Inserimento di **username e password** via Bluetooth
- 💾 Verifica credenziali da un database salvato su **scheda SD**
- 🔓 Apertura automatica della sbarra in caso di accesso corretto
- 🔒 Sistema di sicurezza: timeout, blocco dopo 3 tentativi, reset automatico

</br>

## 🧩 Componenti utilizzati

| Componente         | Descrizione                                               |
|--------------------|-----------------------------------------------------------|
| **ESP32**          | Microcontrollore principale                               |
| **Sensore a ultrasuoni** | Rileva la presenza del veicolo                     |
| **Servomotore**    | Apre e chiude fisicamente la sbarra                      |
| **Modulo Bluetooth** | Per comunicazione con smartphone                        |
| **Scheda SD**      | Contiene il file con il database utenti/password         |
| **Pila da 5V** | Alimentazione aggiuntiva                                    |
| **Breadboard e cavi** | Collegamenti per prototipazione                      |

</br>

---

</br>

## Funzionamento generale

### 🧪 Diagnostica iniziale

All’accensione, il sistema esegue un controllo completo di tutte le componenti:

| Componente        | Controllo                       | In caso di errore                             |
|-------------------|----------------------------------|-----------------------------------------------|
| **Scheda SD**     | Presenza e capacità di lettura   | Messaggio di errore, fallback su memoria RAM  |
| **Sensore ultrasuoni** | Test trigger/echo + stabilità | Blocco del sistema se instabile o assente     |
| **Servomotore**   | Test apertura/chiusura iniziale | Blocco del sistema se non risponde            |
| **Bluetooth**     | Avvio e tentativo di connessione | Messaggio di errore, sistema resta in standby |

</br>

### 🚗 Rilevamento veicolo

Il sensore a ultrasuoni è sempre attivo. Se rileva una macchina a meno di **20 cm**, il sistema attiva la procedura di autenticazione.

</br>

### 🔐 Autenticazione via Bluetooth

- L’ESP32 invia una richiesta via Bluetooth allo smartphone.
- L’utente inserisce `username,password`.
- Il sistema confronta le credenziali con quelle salvate nel file `database.csv` sulla scheda SD.
- Se la SD è assente, si utilizza un database interno temporaneo (in RAM).

</br>

### 📤 Verifica e controllo della sbarra

- ✅ **Accesso corretto:** il servomotore apre la sbarra. Dopo il passaggio del veicolo, si richiude automaticamente.
- ❌ **Accesso errato:** la sbarra resta chiusa. Dopo 3 tentativi falliti consecutivi, il sistema si blocca per alcuni secondi prima di permettere un nuovo accesso.
- ⏱️ **Timeout:** se non vengono inserite le credenziali entro 10 secondi, il sistema si resetta e torna in attesa.

</br>

## 🗂️ Database utenti

Il database è un file CSV (`database.csv`) salvato nella scheda SD:

</br>

## 📲 Interfaccia utente

Si interagisce con il sistema tramite uno smartphone e un'app Bluetooth (es. **Serial Bluetooth Terminal**).

### 📥 Cosa inserire:
- `username` e `password` quando richiesto

### 📤 Cosa ricevi:
- Messaggi chiari con prefissi come `ACCESSO:` o `SISTEMA:` che indicano lo stato attuale

</br>

### 💬 Comandi per diagnostica via Bluetooth:

| Comando           | Descrizione                                        |
|-------------------|----------------------------------------------------|
| `help`            | Mostra tutti i comandi disponibili                 |
| `test`            | Testa tutti i componenti                           |
| `users`           | Mostra gli utenti registrati                       |
| `stats`           | Statistiche generali di funzionamento              |
| `ping`            | Verifica se la connessione Bluetooth è attiva     |
| `status`          | Stato attuale del sistema                          |
| `sensor`          | Diagnostica del sensore a ultrasuoni               |
| `calibrate`       | Calibrazione sensore                               |
| `reset-sensor`    | Resetta le statistiche del sensore                 |
| `sensor-test`     | Esegue un test completo del sensore                |
| `login:usr,pwd`   | Testa il login manualmente (senza veicolo)         |
| `open` / `close`  | Apre/chiude la sbarra manualmente (debug)          |
| `force-reset`     | Forza un reset del sistema                         |

</br>

---

</br> 

## 🧠 Requisiti software

- IDE Arduino con supporto ESP32
- Librerie necessarie:
  - `BluetoothSerial.h`
  - `Servo.h`
  - `SD_MMC.h`
  - (eventualmente `Ultrasonic.h` o codice personalizzato per il sensore)

</br>

---

#### Autori

- Leonardo Pardini 
- Nicola Pannocchia   
- Luca Benedetti
- Pietro Pucci

</br>

---

#### Licenza

Questo progetto è open-source e distribuito sotto licenza **MIT**.

