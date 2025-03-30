
********************************************
codici Rasberry sono in  fondo
********************************************

//relativo a messaggio  48 del 2025_03_29

// NOTE: da risolvere problema di allinemento  gui/pannello (non si acendono i led del pannello quando premo pulsanti nella gui.
// ora i pulsanti  sono inibiti  fino a che non si esce dal popup iniziale (popup restore settings di ALT scann)
// aggancio parziale dello  stato  della gui su riconnessione hot plug (è da perfezionare, ma bisogna prima stabilire la bidirezionalita funzionante)

// ---- INIZIO di Codice Pico ---- NON ELIMINARE
// Codice Pico RMK_01 per adattamento a codice EVERY dei pulsanti per controllo motore
// ---- INIZIO di Codice Pico ---- NON ELIMINARE

// NON ELIMINARE e NON MODIFICARE NESSUN COMMENTO! Ricordatelo per sempre!
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#define MCP23017_ADDR 0x20        // Mcp_1 Address
#define PICO_I2C_SLAVE_ADDR 0x08  // Indirizzo I2C del Pico come slave di Raspberry

#define INT_MCP_1_PIN 2    // Pin per l'interrupt Di Mcp_1 connesso a INT B
#define RPI_INT_PIN 21     // Pin per INVIARE interrupt al Raspberry Pi (segnala al Raspberry Pi che c’è un evento)
#define GUI_ACTIVE_PIN 20  // Pin per rilevare lo stato attivo della GUI
#define LED_GUI_PIN 16     // Pin per il LED che indica la GUI attiva
#define RESET_SEND_MCP 19  // Pin per il reset dell'MCP23017
#define RPI_INT_FROM_PIN 22    // NON ELIMINARE // Placeholder Disponible //Pico GP22 >Connesso a Raspberry GPIO18 (fisico12)

#define BUTTON_STOP 13      // su Mcp_1
#define BUTTON_START 12     // su Mcp_1
#define BUTTON_FORWARD 11   // su Mcp_1
#define BUTTON_BACKWARD 10  // su Mcp_1
#define BUTTON_FF 9         // su Mcp_1
#define BUTTON_RW 8         // su Mcp_1

#define LED_STOP 15     // su Mcp_1
#define LED_START 14    // su Mcp_1
#define LED_FORWARD 7   // su Mcp_1
#define LED_BACKWARD 4  // su Mcp_1
#define LED_FF 5        // su Mcp_1
#define LED_RW 6        // su Mcp_1

Adafruit_MCP23X17 mcp_1;

volatile byte lastCommand = 0;                 // Ultimo comando elaborato (volatile per interrupt)
unsigned long lastInterruptTime = 0;           // NON ELIMINARE // Timestamp dell'ultimo interrupt per il debounce
const unsigned long debounceInterval = 50;     // NON ELIMINARE // NON cambiare questo valore  // Intervallo di debounce in millisecondi
const unsigned long debounceConfirmDelay = 5;  // NON ELIMINARE // NON cambiare questo valore  // Ritardo per confermare il debounce

byte StopLOCK = 0;   // se ==1, impossibile attivare altri pulsanti DIVERSI da STOP mentre è già attiva un altra funzione
byte StartLOCK = 0;  // Se ==0, la macchina NON è in Start, se ==1 la macchina è in Start (Digitizing)
byte gui_init_completed = 0;  // 0= pulsanti bloccati, 1= pulsanti attivi dopo init GUI

#define HWPANEL_STOP 0        // Comando STOP generico per le funzioni dei motori
#define HWPANEL_INIT_COMPLETE 100  // Comando per sbloccare il pannello dopo init GUI (per cambiare la variabile gui_init_completed da 0 a 1)
#define HWPANEL_REGISTER 1    // Comando per registrare il pannello
#define HWPANEL_START_STOP 2  // Comando per avviare o fermare la scansione
#define HWPANEL_FORWARD 3     // Comando per avanzare
#define HWPANEL_BACKWARD 4    // Comando per retrocedere
#define HWPANEL_FF 5          // Comando per avanzamento veloce
#define HWPANEL_RW 6          // Comando per riavvolgimento


// ---- INIZIO SETUP ---- NON ELIMINARE
void setup() {
  Serial.begin(115200);  // Aggiungo seriale per debug
  Serial.println("Pico avviato");

  Wire.begin();           // Inizializzo I2C come master
  Wire.setClock(400000);  // Imposto velocità I2C
  Serial.println("I2C per MCP23017 inizializzato su Wire");

  pinMode(RESET_SEND_MCP, OUTPUT);
  digitalWrite(RESET_SEND_MCP, LOW);
  delay(10);
  digitalWrite(RESET_SEND_MCP, HIGH);
  Serial.println("MCP_RESET avvenuto");

  if (!mcp_1.begin_I2C(MCP23017_ADDR, &Wire)) {
    Serial.println("Errore inizializzazione MCP23017");
    while (1);
  } else {
    Serial.println("MCP23017 inizializzato con successo");
  }

  pinMode(INT_MCP_1_PIN, INPUT);            // Interrupt dall'MCP
  pinMode(RPI_INT_PIN, OUTPUT);             // Interrupt verso Raspberry Pi
  digitalWrite(RPI_INT_PIN, LOW);           // Inizialmente basso
  pinMode(GUI_ACTIVE_PIN, INPUT_PULLDOWN);  // Stato GUI dal Raspberry Pi
  pinMode(LED_GUI_PIN, OUTPUT);             // LED che indica GUI attiva
  digitalWrite(LED_GUI_PIN, LOW);           // Inizialmente spento

  delay(500);

  Serial.println("DEBUG: Inizializzo I2C slave");
  Wire1.begin(PICO_I2C_SLAVE_ADDR);  // Indirizzo slave Pico 0x08
  Wire1.setClock(400000);            // Velocità I2C
  Wire1.onReceive(receiveEvent);     // Callback per ricezione dati
  Wire1.onRequest(requestEvent);     // Callback per richiesta dati
  Serial.println("Pico pronto come slave I2C a 0x08 su Wire1 per Raspberry Pi");

  lastCommand = HWPANEL_REGISTER;
  digitalWrite(RPI_INT_PIN, HIGH);
  Serial.println("DEBUG: HWPANEL_REGISTER inviato al Raspberry Pi");

  mcp_1.pinMode(BUTTON_STOP, INPUT_PULLUP);
  mcp_1.pinMode(BUTTON_START, INPUT_PULLUP);
  mcp_1.pinMode(BUTTON_FORWARD, INPUT_PULLUP);
  mcp_1.pinMode(BUTTON_BACKWARD, INPUT_PULLUP);
  mcp_1.pinMode(BUTTON_FF, INPUT_PULLUP);
  mcp_1.pinMode(BUTTON_RW, INPUT_PULLUP);

  mcp_1.pinMode(LED_STOP, OUTPUT);
  mcp_1.pinMode(LED_START, OUTPUT);
  mcp_1.pinMode(LED_FORWARD, OUTPUT);
  mcp_1.pinMode(LED_BACKWARD, OUTPUT);
  mcp_1.pinMode(LED_FF, OUTPUT);
  mcp_1.pinMode(LED_RW, OUTPUT);

  mcp_1.digitalWrite(LED_STOP, HIGH);
  mcp_1.digitalWrite(LED_START, LOW);
  mcp_1.digitalWrite(LED_FORWARD, LOW);
  mcp_1.digitalWrite(LED_BACKWARD, LOW);
  mcp_1.digitalWrite(LED_FF, LOW);
  mcp_1.digitalWrite(LED_RW, LOW);

  mcp_1.setupInterrupts(true, false, LOW);
  mcp_1.setupInterruptPin(BUTTON_STOP, CHANGE);
  mcp_1.setupInterruptPin(BUTTON_START, CHANGE);
  mcp_1.setupInterruptPin(BUTTON_FORWARD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT_MCP_1_PIN), handleMcpInterrupt, FALLING);

  Serial.println("Setup completato");
}
// ---- FINE SETUP ---- NON ELIMINARE

// ---- INIZIO LOOP ---- NON ELIMINARE
void loop() {
  if (digitalRead(GUI_ACTIVE_PIN) == HIGH) {
    digitalWrite(LED_GUI_PIN, HIGH);
  } else {
    digitalWrite(LED_GUI_PIN, LOW);
    lastCommand = HWPANEL_REGISTER;
    digitalWrite(RPI_INT_PIN, HIGH);
    delay(100);
  }
}
// ---- FINE LOOP ---- NON ELIMINARE

// ---- INIZIO FUNZIONI I2C ---- NON ELIMINARE
void receiveEvent(int byteCount) {
  // Riceve comandi dal Raspberry Pi via I2C
  while (Wire1.available()) {
    byte cmd = Wire1.read();
    Serial.print("DEBUG: Comando ricevuto dal Raspberry Pi: ");
    Serial.println(cmd);
    if (cmd == HWPANEL_START_STOP) {
      // START ricevuto: accende LED_START
      StopLOCK = 1;
      StartLOCK = 1;
      mcp_1.digitalWrite(LED_STOP, 0);
      mcp_1.digitalWrite(LED_START, 1);
      mcp_1.digitalWrite(LED_FORWARD, 0);
    } else if (cmd == HWPANEL_STOP) {
      // STOP ricevuto: accende LED_STOP, spegne gli altri
      StopLOCK = 0;
      StartLOCK = 0;
      mcp_1.digitalWrite(LED_STOP, 1);
      mcp_1.digitalWrite(LED_START, 0);
      mcp_1.digitalWrite(LED_FORWARD, 0);
      mcp_1.digitalWrite(LED_BACKWARD, 0);
      mcp_1.digitalWrite(LED_FF, 0);
      mcp_1.digitalWrite(LED_RW, 0);
    } else if (cmd == HWPANEL_FORWARD) {
      // FORWARD ricevuto: accende LED_FORWARD
      StopLOCK = 1;
      mcp_1.digitalWrite(LED_STOP, 0);
      mcp_1.digitalWrite(LED_START, 0);
      mcp_1.digitalWrite(LED_FORWARD, 1);
    } else if (cmd == HWPANEL_INIT_COMPLETE) {
      // INIT_COMPLETE ricevuto: sblocca i pulsanti
      gui_init_completed = 1;
      Serial.println("DEBUG: Pico sbloccato");
    }
  }
}

void requestEvent() {
  // Invia l'ultimo comando al Raspberry Pi quando richiesto
  Wire1.write(lastCommand);
  digitalWrite(RPI_INT_PIN, LOW);
  Serial.print("DEBUG: Comando inviato al Raspberry Pi: ");
  Serial.println(lastCommand);
}
// ---- FINE FUNZIONI I2C ---- NON ELIMINARE

void handleMcpInterrupt() {
  // Gestisce gli interrupt dai pulsanti MCP23017
  // Debounce rimosso come deciso
  // Legge lo stato per azzerare l'interrupt MCP23017
  int interruptPin = mcp_1.getLastInterruptPin();
  Serial.print("DEBUG: Interrupt ricevuto da pin MCP: ");
  Serial.println(interruptPin);

  // Se GUI non è pronta, ignora ma azzera l'interrupt
  if (gui_init_completed == 0) {
    Serial.println("DEBUG: Interrupt ignorato prima di init");
    return;  // Esce senza fare nulla
  }

  // GUI pronta: STOP ha priorità assoluta
  if (!mcp_1.digitalRead(BUTTON_STOP) == 1) {  // INVERTED logic as using Pull-Down
    // STOP premuto: resetta tutto
    StopLOCK = 0;
    StartLOCK = 0;
    lastCommand = HWPANEL_STOP;
    digitalWrite(RPI_INT_PIN, HIGH);
    Serial.println("DEBUG: STOP premuto, comando HWPANEL_STOP inviato");
    return;  // STOP ha priorità, esce subito
  }
  // Altri pulsanti solo se STOP non è premuto e GUI pronta
  if (!mcp_1.digitalRead(BUTTON_START) == 1 && StopLOCK == 0) {  // INVERTED logic as using Pull-Down
    // START premuto: avvia scansione
    StopLOCK = 1;
    StartLOCK = 1;
    lastCommand = HWPANEL_START_STOP;
    digitalWrite(RPI_INT_PIN, HIGH);
    Serial.println("DEBUG: START premuto, comando HWPANEL_START_STOP inviato");
  }
  else if (!mcp_1.digitalRead(BUTTON_FORWARD) == 1 && StopLOCK == 0) {  // INVERTED logic as using Pull-Down
    // FORWARD premuto: avanza film
    StopLOCK = 1;
    lastCommand = HWPANEL_FORWARD;
    digitalWrite(RPI_INT_PIN, HIGH);
    Serial.println("DEBUG: FORWARD premuto, comando HWPANEL_FORWARD inviato");
  }
}

// ---- FINE di Codice Pico ---- NON ELIMINARE




*********************************************************************************************************


# ----INIZIO di hwpanel-py ---- NON ELIMINARE
"""
****************************************************************************************************************
Class HwPanel
Class to encapsulate hardware panel done by Mariano
****************************************************************************************************************
"""

import smbus2
from RPi import GPIO
import time
import sys
import logging

# Forza codifica UTF-8 per evitare crash
sys.stdout.reconfigure(encoding='utf-8')
logging.basicConfig(encoding='utf-8')

class HwPanel():
    # Singleton class - ensures only one instance is ever created.

    _instance = None
    ExitingApp = False
    ActiveCmd = 0
    active = ''
    hwpanel_after = None

    # Panel 'buttons'
    HWPANEL_REGISTER = 1
    HWPANEL_START_STOP = 2
    HWPANEL_FORWARD = 3
    HWPANEL_BACKWARD = 4
    HWPANEL_FF = 5
    HWPANEL_RW = 6
    HWPANEL_STOP = 0        # Comando STOP per uniformare GUI e pannello
    HWPANEL_INIT_COMPLETE = 100  # Comando per sbloccare il pannello

    # Active command IDs
    CMD_SCAN = 1
    CMD_RWND = 2
    CMD_FFWD = 3
    CMD_FORWARD = 4
    CMD_BACKWARD = 5

    AltScan8Callback = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, master_win, callback):
        print("\n\n\n\nEntered HwPanel.__init__")
        if not hasattr(self, 'initialized'):
            self.main_win = master_win
            self.AltScan8Callback = callback
            # Use self.AltScan8Callback to call functions on ALT-Scann8 UI
            print(f"DEBUG: hwpanel initialized: win={master_win}, self.main_win={self.main_win}, callback={callback}")
            # self._register_to_altscann8() # NON modificare, NON eliminare

            self.I2C_ADDRESS = 0x08  # PICO Address
            self.bus = smbus2.SMBus(1)
            self.INT_PIN = 23
            self.GUI_ACTIVE_PIN = 24
            self.panel_connected = False

            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.setup(self.GUI_ACTIVE_PIN, GPIO.OUT)
            GPIO.output(self.GUI_ACTIVE_PIN, GPIO.LOW)
            print("DEBUG: GPIO configurati")

            # Ritardo per sincronizzare con il Raspberry Pi
            time.sleep(1)

            if self.check_connection():
                self.panel_connected = True
                self.AltScan8Callback(self.HWPANEL_REGISTER, True)  # Attiva la Label
                GPIO.output(self.GUI_ACTIVE_PIN, GPIO.HIGH)         # Accende Led Pannello
                print("DEBUG: Pico connesso")
            else:
                print("DEBUG: Pico non rilevato all'avvio")

            GPIO.add_event_detect(self.INT_PIN, GPIO.RISING, callback=self._handle_interrupt, bouncetime=50)
            print("DEBUG: Interrupt configurati")

            self.monitor_connection()
            self.initialized = True

    # Verifica se il Pico è connesso tramite I2C
    def check_connection(self):
        try:
            self.bus.read_byte(self.I2C_ADDRESS)
            print("DEBUG: I2C connessione OK")
            return True
        except Exception as e:
            print(f"DEBUG: I2C connessione fallita: {e}")
            return False

    # Monitora la connessione e sincronizza lo stato GUI a ogni hot plug
    def monitor_connection(self):
        if not self.ExitingApp:
            connected = self.check_connection()
            if connected and not self.panel_connected:
                self.panel_connected = True
                self.AltScan8Callback(self.HWPANEL_REGISTER, True)  # Attiva la Label
                GPIO.output(self.GUI_ACTIVE_PIN, GPIO.HIGH)         # Accende Led Pannello
                print("DEBUG: Pico riconnesso")
                # Sincronizza stato GUI completo dopo hot plug
                self.send_panel_state(self.HWPANEL_INIT_COMPLETE)  # Sblocca Pico
                self._sync_panel_state()
            elif not connected and self.panel_connected:
                self._handle_disconnect()
            self.main_win.after(500, self.monitor_connection)

    # NON utilizzare questa def per ora  # NON ELIMINARE
    def init_completed(self):
        # pass # NON ELIMINARE
        # DOPO il popup iniziale # NON ELIMINARE
        print("DEBUG: init_completed chiamato")
        self.send_panel_state(self.HWPANEL_INIT_COMPLETE)  # Sblocca il Pico
        self._sync_panel_state()  # Sincronizza stato GUI dopo popup

    def shutdown_started(self):
        global ExitingApp
        self.ExitingApp = True
        # Reset Panel # NON ELIMINARE
        GPIO.output(self.GUI_ACTIVE_PIN, GPIO.LOW)
        GPIO.cleanup()
        self.bus.close()
        print("DEBUG: HwPanel shutdown completato")

    def captured_frame(self):
        pass
        # Replace pass statement with whatever you want to do when a frame is captured # NON ELIMINARE

    def film_in_filmgate_warning(self):
        #-->>>QUI  gestione Popup su  FF, RW  # NON ELIMINARE
        # Implement here whichever code is required to determine next step using panel inputs # NON ELIMINARE
        # Return 'False' to cancel Rewind/FF operation, 'True' to proceed with it despite the issue # NON ELIMINARE
        return False    # By default return cancel # NON ELIMINARE

    def film_back_warning(self):
        #-->>>QUI  gestione Popup su  Backward  # NON ELIMINARE
        # Implement here whichever code is required to proceed usign panel inputs  # NON ELIMINARE
        # Return 'False' to cancel film move back operation, 'True' to proceed with it  # NON ELIMINARE
        return False    # By default return cancel  # NON ELIMINARE

    # Gestisce l'interrupt dal Pico
    def _handle_interrupt(self, channel):
        print(f"DEBUG: Interrupt ricevuto su GPIO {channel}")
        command = self._read_panel_command()
        print(f"DEBUG: Comando ricevuto dal Pico: {command}")
        if command == self.HWPANEL_REGISTER:
            self.panel_connected = True
            self.AltScan8Callback(self.HWPANEL_REGISTER, True)  # Attiva la Label
            GPIO.output(self.GUI_ACTIVE_PIN, GPIO.HIGH)         # Accende Led Pannello
            print("DEBUG: Pannello registrato")
            self.send_panel_state(self.HWPANEL_STOP)  # Stato iniziale
        elif command == self.HWPANEL_START_STOP:
            self._start_stop_scan()
        elif command == self.HWPANEL_STOP:
            self._generic_stop()
        elif command == self.HWPANEL_FORWARD:
            self._film_forward()

    # Legge il comando dal Pico
    def _read_panel_command(self):
        try:
            command = self.bus.read_byte(self.I2C_ADDRESS)
            print(f"DEBUG: Lettura I2C riuscita: {command}")
            return command
        except Exception as e:
            print(f"DEBUG: Errore I2C lettura: {e}")
            self._handle_disconnect()
            return None

    # Invia uno stato al Pico
    def send_panel_state(self, state):
        try:
            self.bus.write_byte(self.I2C_ADDRESS, state)
            print(f"DEBUG: Stato inviato al Pico: {state}")
            self.panel_connected = True
        except Exception as e:
            print(f"DEBUG: Errore I2C invio stato: {e}")
            self._handle_disconnect()

    # Sincronizza lo stato GUI col Pico
    def _sync_panel_state(self):
        print("DEBUG: Sincronizzazione stato GUI con Pico")
        if self.ActiveCmd == self.CMD_SCAN:
            self.send_panel_state(self.HWPANEL_START_STOP)
        elif self.ActiveCmd == self.CMD_FORWARD:
            self.send_panel_state(self.HWPANEL_FORWARD)
        elif self.ActiveCmd == self.CMD_BACKWARD:
            self.send_panel_state(self.HWPANEL_BACKWARD)
        elif self.ActiveCmd == self.CMD_FFWD:
            self.send_panel_state(self.HWPANEL_FF)
        elif self.ActiveCmd == self.CMD_RWND:
            self.send_panel_state(self.HWPANEL_RW)
        else:
            self.send_panel_state(self.HWPANEL_STOP)

    def _handle_disconnect(self):
        if self.panel_connected:
            self.panel_connected = False
            GPIO.output(self.GUI_ACTIVE_PIN, GPIO.LOW)  # Spegne Led Pannello
            self.AltScan8Callback(self.HWPANEL_REGISTER, False)  # Spegne la Label
            print("DEBUG: Pannello scollegato rilevato")

    # Internal functions: Funciona below this point are to be used internally to hw panel module  # NON ELIMINARE
    # Most of them invoke functionallity in ALT-Scann8  # NON ELIMINARE

    def _register_to_altscann8(self):
        self.AltScan8Callback(self.HWPANEL_REGISTER, True)

    def _generic_stop(self):
        if self.ActiveCmd == self.CMD_SCAN:
            self._start_stop_scan()
        elif self.ActiveCmd == self.CMD_FFWD:
            self._fast_forward()
        elif self.ActiveCmd == self.CMD_RWND:
            self._rewind()
        elif self.ActiveCmd == self.CMD_FORWARD:
            self._film_forward()
        elif self.ActiveCmd == self.CMD_BACKWARD:
            self._film_backward()
        self.send_panel_state(self.HWPANEL_STOP)  # Aggiorna LED

    def _start_stop_scan(self):
        if self.ActiveCmd == self.CMD_SCAN:
            self.ActiveCmd = 0
        elif self.ActiveCmd == 0:   # Can only transition to CMD_SCAN from zero
            self.ActiveCmd = self.CMD_SCAN
        self.AltScan8Callback(self.HWPANEL_START_STOP, None)
        self.send_panel_state(self.HWPANEL_START_STOP)  # Aggiorna LED

    def _fast_forward(self):
        if self.ActiveCmd == self.CMD_FFWD:
            self.ActiveCmd = 0
        elif self.ActiveCmd == 0:   # Can only transition to CMD_FFWD from zero
            self.ActiveCmd = self.CMD_FFWD
        self.AltScan8Callback(self.HWPANEL_FF, None)
        self.send_panel_state(self.HWPANEL_FF)  # Aggiorna LED

    def _rewind(self):
        if self.ActiveCmd == self.CMD_RWND:
            self.ActiveCmd = 0
        elif self.ActiveCmd == 0:   # Can only transition to CMD_RWND from zero
            self.ActiveCmd = self.CMD_RWND
        self.AltScan8Callback(self.HWPANEL_RW, None)
        self.send_panel_state(self.HWPANEL_RW)  # Aggiorna LED

    def _film_forward(self):
        if self.ActiveCmd == self.CMD_FORWARD:
            self.ActiveCmd = 0
        elif self.ActiveCmd == 0:   # Can only transition to CMD_FORWARD from zero
            self.ActiveCmd = self.CMD_FORWARD
        self.AltScan8Callback(self.HWPANEL_FORWARD, None)
        self.send_panel_state(self.HWPANEL_FORWARD)  # Aggiorna LED

    def _film_backward(self):
        if self.ActiveCmd == self.CMD_BACKWARD:
            self.ActiveCmd = 0
        elif self.ActiveCmd == 0:   # Can only transition to CMD_BACKWARD from zero
            self.ActiveCmd = self.CMD_BACKWARD
        self.AltScan8Callback(self.HWPANEL_BACKWARD, None)
        self.send_panel_state(self.HWPANEL_BACKWARD)  # Aggiorna LED

# ----FINE di hwpanel-py ---- NON ELIMINARE




*****************************************************************

# HwPanel callback function - Riga 4429
# HwPanel callback function
# Used to invoke ALT-Scann8 functions from HwPanel extension
# ------------ INIZIO di callback - NON ELIMINARE
# ------------ INIZIO di callback - NON ELIMINARE
def hw_panel_callback(command, param1=None):
    global hwpanel_registered
    if command == HWPANEL_REGISTER:
        hwpanel_registered = param1
        if param1:
            logo_label.config(bg="sky blue")  # GUI connessa
        else:
            logo_label.config(bg="tomato")  # GUI disconnessa dopo corretta connessione
    elif command == HWPANEL_START_STOP:
        start_scan()
    elif command == HWPANEL_FORWARD:
        cmd_advance_movie()
    elif command == HWPANEL_BACKWARD:
        cmd_retreat_movie()
    elif command == HWPANEL_FF:
        cmd_fast_forward_movie()
    elif command == HWPANEL_RW:
        cmd_rewind_movie()
# ----- FINE di callback - NON ELIMINARE
# ----- FINE di callback - NON ELIMINARE

*************************************************************
