#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"

/***
 * 
 *  Driver for MIDI concertina running on Adafruit's Bluefruit 32u4.
 * 
 *  The concertina's keyboard consists of a 14-column by 4-row matrix of buttons. Each button 
 *  consists of a normally open pushbutton switch in series with a diode. The anode of the 
 *  diode is connected to the button's row. Its cathode is connected to the switch. The other 
 *  end of the switch is connected to the button's column. Each row is connected to a digital 
 *  input pin on the Bluefruit set to INPUT_PULLUP mode. Each column is connected to one of the 
 *  output pins on one of two HC164 8-bit shift registers (of which only 7 bits are used). The 
 *  HC164s are connected in series, effectively making them into a 14-bit shift register. One 
 *  HC164 is used for the 28 buttons on the left side of the concertina  and the other for the 
 *  28 buttons on the concertina's right side.
 * 
 *       ---------------|
 *       |              |
 *       |              |
 *       --> |  Push    |
 *       --> |  Switch  |
 *       |              |
 *     --.--            |
 *      / \    Diode    |
 *     -----            |
 *       |              |
 *  --------------------{------ Row (to digital input pin on Arduino)
 *                      |
 *                   Column (to output pin of HC164)
 * 
 *  When a columns's HC164 output pin is HIGH, the column is deselected. That is, since its 
 *  row (like all the rows) is in INPUT_PULLUP mode, there is no voltage difference between 
 *  the anodes of the column's buttons and the column's output pin. That means pushing any 
 *  of them has no effect. On the other hand, setting a column's HC164 output pin to LOW 
 *  selects it. This is because doing so creates a voltage difference between column's 
 *  buttons' anodes and the far ends of each of the switches. If a button's switch is pushed, 
 *  that button's row is pulled to ground and can be detected by the Bluefruit.
 * 
 *  On each trip through loop() the sketch processes the buttons in one column -- the selected 
 *  column. Column selection is done as follows. When it is time to sample the first column, 
 *  the sketch injects a LOW into the input of the HC164 for the left side of the concertina 
 *  and strobes the HC164's clock input. This selects the first column. On subsequent trips 
 *  through loop(), a HIGH is injected into the HC164's input and the clock is strobed. This
 *  shifts the LOW one column farther along, selecting it. After 14 trips through loop(), the 
 *  whole process starts again with the first column.
 * 
 *  Once the column for this trip through loop() has been selected, the state of each button
 *  in that column is sampled: If the input pin corresponding to a button is LOW, the button
 *  is down. If it's HIGH, the button is up.
 * 
 *  The sketch keeps track of the state of the buttons as they are scanned. If a button's 
 *  state changes and the Bluefruit's BLE radio is connected (presumably to a MIDI synth), the 
 *  sketch uses the Adafruit BLE library to send the appropriate MIDI message using the radio.
 * 
 * The concertina also contains a differential pressure sensor that measures the difference 
 * between the pressure on the inside and outside of the bellows. It is read as an analog value 
 * on pin P_SENSE. The absoulte value of the pressure difference between the inside and outside 
 * of the bellows tells how hard the bellows is being worked. This value is used to to set the 
 * MIDI expression pedal (instrument volume) value via a pressure-to-expression transfer 
 * function. Approximately every EXP_MSG_MS milliseconds the pressure is read, transformed 
 * to an expression value and sent via MIDI as an expression ppedal value.
 * 
 * =====
 *
 *  @file     main.cpp 
 * 
 *  @version  Version 0.3.5, April 2022
 *
 *  @author   D. L. Ehnebuske
 *
 *  @section  license
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-2022 by D. L. Ehnebuke All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *    1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 *    2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 *    3. Neither the name of the copyright holders nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***/

/*
 * GPIO Pin assignments
 */
#define CK                      (2)     // HC164 clock pin
#define D_IN                    (3)     // HC164 serial input pin
#define ROW_BASE                (20)    // First row is attached to this pin
#define A                       (20)    // The individual rows
#define B                       (21)
#define C                       (22)
#define D                       (23)
#define LED_R                   (9)     // Status LED pins (must be PWM)
#define LED_G                   (10)
#define LED_B                   (11)
#define P_SENSE                 (A0)    // Pressure sensor analog input

/*
* Adafruit Bluefruit constants
*/
#define BUFSIZE                 (160)   // Size of the read buffer for incoming data
#define BLUEFRUIT_UART_MODE_PIN (-1)    // Set to 12 if used
#define BLUEFRUIT_SPI_CS        (8)
#define BLUEFRUIT_SPI_IRQ       (7)
#define BLUEFRUIT_SPI_RST       (4)     // Optional but recommended, set to -1 if unused

/*
 * 	Bézier pressure to MIDI expression transfer function constants
 * 
 *  A Bézier curve, S0, is defined as: s0 = (1 − t)^3 * P0 + 3 * (1 − t)^2 * t * P1 + 3 * (1 − t) * t^2 * P2 + t^3 * P3
 *  Where t∈[0,1]. See http://www.malinc.se/m/DeCasteljauAndBezier.php
 */
#define EXP_MIN         (50)            // MIDI "Expression" minimum value
#define EXP_MAX         (127)           // MIDI "Expression" maximum value
#define PRS_MIN         (1)             // Minimum getPressure() pressure sensor value
#define PRS_MAX         (120)           // Maximum getPressure() pressure sensor value
#define P0_X            (0)             // Anchor point 1
#define P0_Y            (EXP_MIN)
#define P1_X            (10)            // Control point 1
#define P1_Y            (EXP_MIN)
#define P2_X            (15)            // Control point 2
#define P2_Y            (EXP_MAX)
#define P3_X            (PRS_MAX)       // Anchor point 2
#define P3_Y            (EXP_MAX)

#define BC_STEP         (0.005)         // Step size for t
#define TF_COUNT        (PRS_MAX + 1)   // Count of transfer function values

#define EXP_MAX_ERR     (5)             // Max tolerable expresson error (integer tenths of a percent)
#define EXP_MAX_STEP    (20)            // The maximum one-step change in expression (integer tenths of a percent)
#define EXP_MIN_MILLIS  (35)            // Won't send expression pedal messages more frequently than this
#define EXP_DUMP_MILLIS (10000)         // For debugging: interval (ms) for dumping expression pedal info to Serial

/*
 * Other symbolic constants
 */
#define BANNER                  F("Bluefruit concertina driver v0.3.6")
//#define VERBOSE_MODE                    // Uncomment to enable debug output
//#define DEBUG_EXP                       // Uncomment to enable expression pedal debugging
#define COL_COUNT               (14)    // Number of columns in matrix
#define ROW_COUNT               (4)     // Number of rows in matrix
#define COL_SEL                 (LOW)   // A column is selected when its pin is grounded
#define COL_DSEL                (HIGH)
#define SW_UP                   (false) // Status of switches -- up or down
#define SW_DN                   (true)
#define MIDI_NOTE_ON            (0x90)  // MIDI message "note on" status code
#define MIDI_CTRL_CHG           (0xB0)  // MIDI message "control change" status code
#define EXP_CTRL                (11)    // MIDI "expression" control number
#define STATUS_DISCO            (0)     // Disconnected
#define STATUS_CONND            (1)     // Connected
#define STATUS_ERROR            (2)     // Error (See Serial output)

/*
 * Global variables
 */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
                                          // The object representing the Bluetooth LE radio
Adafruit_BLEMIDI midi(ble);               // The object representing the MIDI target device
bool isConnected = false;                 // The state of the BLE connection

int curCol = 0;                           // Matrix column to be scanned at next pass through loop()
bool sw[ROW_COUNT][COL_COUNT] = {SW_UP};  // Switch status last time we looked
uint8_t midiNote[ROW_COUNT][COL_COUNT] =  // Corresponding MIDI note numbers
  {{54,  50,  61,  71,  81,  78,  88,   90,  93,  82,  72,  62,  51,  55},
   {68,  64,  75,  84,  95,  92,  56,   75,  79,  68,  59,  48,  96,  87},
   {80,  77,  87,  53,  51,  60,  70,   63,  65,  56,  89,  80,  83,  73},
   {94,  91,  57,  67,  63,  74,  85,   49,  52,  86,  76,  66,  69,  58}};
String name = "ABCDEFGABCDEFG";

unsigned long expMillis = 0;              // millis() when the last expression pedal value MIDI msg was sent
uint8_t expCount = 0;                     // The MIDI expression pedal value sample counter
uint32_t expAccum = 0;                    // Accumulator for expression readings
int initialPressure = 0;                  // Initial raw pressure reading from sensor
int expVal = EXP_MIN;                     // MIDI expression value
uint8_t tfVal[TF_COUNT];                  // Expression transfer function look-up table

uint8_t statusVal[3][3] =                 // LED RGB values for each of the STATUS_????? values
  {{200, 180, 000},                       //  STATUS_DISCO
   {000, 255, 000},                       //  STATUS_CONND
   {255, 000, 000}};                      //  STATUS_ERROR

/*
 * Get differential pressure sensor reading
*/
int getPress(){
  int answer = 0;                         // 0 if not connected
  if (isConnected) {
    answer = abs((analogRead(P_SENSE)) - initialPressure);
    answer = constrain(answer, PRS_MIN, PRS_MAX);
  }
  return answer;
}

/*
 * Set Status LED
 */
void setStatus(uint8_t status) {
  analogWrite(LED_R, statusVal[status][0]);
  analogWrite(LED_G, statusVal[status][1]);
  analogWrite(LED_B, statusVal[status][2]);
}

/*
 * Fatal error handler.
 */
void error(const __FlashStringHelper* err) {
  Serial.println(err);
  setStatus(STATUS_ERROR);
  while (true);
}

/*
 * Callback for "connected" BLE event 
 */
void connected(void) {
  isConnected = true;
  #ifdef VERBOSE_MODE
    Serial.println(F("Connected."));
  #endif
  setStatus(STATUS_CONND);

  // Read the initial bellows pressure. 
  // This is the reference against which we measure
  analogRead(P_SENSE);                    // Emprirically, pressure sensor first reading is bogus
  delay(1000);                            // And it needs some time to get its act together
  initialPressure = analogRead(P_SENSE);  // But after a second it's good to go
  #ifdef VERBOSE_MODE
    Serial.print(F("Initial pressure: "));
    Serial.println(initialPressure);
  #endif
  // initialize the shift register clock and data input
  digitalWrite(CK, LOW);
  digitalWrite(D_IN, COL_DSEL);
  // Deselect all switch matrix columns
  for (int col = 0; col < COL_COUNT; col++) {
    digitalWrite(CK, HIGH);
    digitalWrite(CK, LOW);
  }
}

/*
 * Callback for "disconnected" BLE event
 */
void disconnected(void) {
  isConnected = false;
  #ifdef VERBOSE_MODE
    Serial.println("Disconnected.");
  #endif
  setStatus(STATUS_DISCO);
}

/*
 * Callback for "MIDI message received" BLE event (It's ignored.)
 */
void bleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2) {
    Serial.print(F("[MIDI "));
    Serial.print(timestamp);
    Serial.print(F(" ] "));
    Serial.print(status, HEX);
    Serial.print(F(" "));
    Serial.print(byte1 , HEX);
    Serial.print(F(" "));
    Serial.println(byte2 , HEX);
}

/*
 * The usual Arduino setup() function. Called once by the environment at start-up or reset
 */
void setup() {
  // Say hello
  Serial.begin(9600);
  delay(4000);
  Serial.println(BANNER);

  // Set up status LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setStatus(STATUS_DISCO);

  // Initialize pressure-to-expression transfer function lookup table.
  // (Not at all efficient, but easy and only done once.)
  float t = -BC_STEP;
  for (int ix = 0; ix < TF_COUNT; ix++) {
    uint8_t tfX;
    uint8_t tfY;
    do {
      if (t > 1.0) {
        error(F("Bézier curve transfer function initialization failed."));
      }
      t += BC_STEP;
      float a = t * t;                    // t^2
      float b = a * t;                    // t^3
      float c = (1 - t) * (1 - t);        // (1 - t)^2
      float d = c * (1 - t);              // (1 - t)^3
      tfX = (uint8_t)(d * P0_X + 3 * c * t * P1_X + 3 * (1 - t) * a * P2_X + b * P3_X + 0.5);
      tfY = (uint8_t)(d * P0_Y + 3 * c * t * P1_Y + 3 * (1 - t) * a * P2_Y + b * P3_Y + 0.5);
    } while(tfX < ix);
    tfVal[ix] = tfY;
    t -= BC_STEP;
  }
  #ifdef VERBOSE_MODE
    Serial.println(F("Bellows pressure to MIDI expression lookup table."));
    for (int ix = 0; ix < TF_COUNT; ix++) {
      Serial.print(F("("));
      Serial.print(ix);
      Serial.print(F(", "));
      Serial.print(tfVal[ix]);
      Serial.print(F(") "));
      if ((ix + 1) % 10 == 0) {
        Serial.println(F(""));
      }
    }
    Serial.println(F(""));
  #endif

  // Initialize the Bluetooth radio module
  #ifdef VERBOSE_MODE
    Serial.print(F("Initialising Bluefruit BLE module: "));
  #endif
  if (!ble.begin(
    #ifdef VERBOSE_MODE 
      true 
    #else 
      false 
    #endif
    )) {
    error(F("Couldn't find BLE module."));
  }
  #ifdef VERBOSE_MODE
    Serial.println(F("OK!"));
  #endif

  // Perform a factory reset to make sure everything is in a known state
  #ifdef VERBOSE_MODE
    Serial.println(F("Performing a factory reset: "));
  #endif
  if (!ble.factoryReset()) {
    error(F("Couldn't factory reset"));
  }
  ble.echo(false);

  #ifdef VERBOSE_MODE
    Serial.println("Bluetooth radio module info:");
    ble.info();
  #endif

  // Set BLE callbacks
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(bleMidiRX);

  // Enable MIDI
  #ifdef VERBOSE_MODE
    Serial.println(F("Enable MIDI: "));
  #endif
  if (!midi.begin(true)) {
    error(F("Could not enable MIDI."));
  }

  ble.verbose(false);
  #ifdef VERBOSE_MODE
    Serial.print(F("Waiting for a connection..."));
  #endif

  // Set up the shift register control pins
  pinMode(CK, OUTPUT);
  digitalWrite(CK, LOW);
  pinMode(D_IN, OUTPUT);
  digitalWrite(D_IN, COL_DSEL);

  // Set up the row input pins
  for (int row = 0; row < ROW_COUNT; row++) {
    pinMode(ROW_BASE + row, INPUT_PULLUP);
  }
}

/*
 * The usual Arduino loop() function. Called repeatedly by the environment after setup() returns.
 */
void loop() {
  
  // Check for new BLE and MIDI RX events (handled by callbacks)
  ble.update(500);

  // Bail if BLE not connected
  if (!isConnected) {
    return;
  }

  // Set expression pedal (instrument volume) based on bellows pressure. Set it a maximum of once 
  // every EXP_MIN_MILLIS ms and even then only if the error betwen what it's currently set to 
  // and what the bellows pressure indicates it shuld be differs by more than EXP_MAX_ERR integer 
  // percent. But don't change it by more than EXP_MAX_STEP integer percent. The idea is to set it 
  // often enough to rrasonably track the bellows pressure without using up all the MIDI BLE
  // bandwidth and to do it smoothly enough to avoid audible jumps in volume.
  unsigned long nowMillis = millis();
  #ifdef DEBUG_EXP
    static int expCount = 0;
    static unsigned long dumpMillis = 0;
    if (dumpMillis == 0) {
      dumpMillis = nowMillis;
    }
  #endif
  if (nowMillis - expMillis > EXP_MIN_MILLIS) {
    int nowExpVal = tfVal[getPress()];
    int pctChangeX10 = (abs(nowExpVal - expVal) * 1000) / expVal;
    if (pctChangeX10 > EXP_MAX_ERR) {
      if (pctChangeX10 > EXP_MAX_STEP) {
        if (nowExpVal > expVal) {
          nowExpVal = min(expVal + ((EXP_MAX_STEP * 10) / expVal), EXP_MAX);
        } else {
          nowExpVal = max(expVal - ((EXP_MAX_STEP * 10) / expVal), EXP_MIN);
        }
      }
      #ifdef DEBUG_EXP
        expCount++;
      #endif
      midi.send(MIDI_CTRL_CHG, EXP_CTRL, nowExpVal);
      expVal = nowExpVal;
      #ifdef VERBOSE_MODE
        Serial.print(F("Expression "));
        Serial.println(expVal);
      #endif
    }
    expMillis = nowMillis;
  }
  #ifdef DEBUG_EXP
    if (nowMillis - dumpMillis > EXP_DUMP_MILLIS) {
      Serial.print(F("Expression pedal message rate: "));
      Serial.print(expCount * 1000.0 / EXP_DUMP_MILLIS);
      Serial.println(" msg/sec.");
      expCount = 0;
      dumpMillis = nowMillis;
    }
  #endif

  // Advance selected column to curCol. The hardware consists of two HC164 shift registers configured
  // to act as a single 14-bit shift register. Selecting a column consists of making that column's output
  // be the value COL_SEL and all the others be COL_DSEL. Initially all the values are set to COL_DSEL.
  // When curCol is 0 we shift a COL_SEL into the first (column 0) position, which shifts all the values
  // along one position. The value for the last position "falls off the end" and is lost. In subsequest 
  // trips through, we shift a COL_DSEL into the first position, and shift everything (including the 
  // previously injected COL_SEL) one position along.
  if (curCol == 0) {
    digitalWrite(D_IN, COL_SEL);
    digitalWrite(CK, HIGH);
    digitalWrite(CK, LOW);
    digitalWrite(D_IN, COL_DSEL);
  } else {
    digitalWrite(CK, HIGH);
    digitalWrite(CK, LOW);
  }

  // Deal with buttons in curCol
  bool curColSw[ROW_COUNT];
  for (int row = 0; row < ROW_COUNT; row++) {
    curColSw[row] = digitalRead(row + ROW_BASE) == LOW ? SW_DN : SW_UP;

    // Deal with a button whose state changed
    if (curColSw[row] != sw[row][curCol]) {
      sw[row][curCol] = curColSw[row];
      midi.send(MIDI_NOTE_ON, midiNote[row][curCol], curColSw[row] == SW_DN ? 127 : 0);
      #ifdef VERBOSE_MODE
        Serial.print(F("Button "));
        Serial.print(name.charAt(curCol));
        Serial.print(name.charAt(row));
        Serial.print(F(" which is note "));
        Serial.print(midiNote[row][curCol]);
        if (curColSw[row] == SW_DN) {
          Serial.println(F(" on"));
        } else {
          Serial.println(F(" off"));
        }
      #endif
    }
  }

  // Advance to next column
  if (++curCol >= COL_COUNT) {
    curCol = 0;
  }
}
