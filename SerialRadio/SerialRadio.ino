/// RDA5807FP I²S Output FM Tuner working with ESP32-WROOM-32D
/// \file SerialRadio.ino 
/// \brief Radio implementation using the Serial communication.
/// 
/// \author Matthias Hertel, http://www.mathertel.de
/// \copyright Copyright (c) 2014 by Matthias Hertel.\n
/// This work is licensed under a BSD style license.\n
/// See http://www.mathertel.de/License.aspx
///
/// \details
/// This is a full function radio implementation that uses a LCD display to show the current station information.\n
/// It can be used with various chips after adjusting the radio object definition.\n
/// Open the Serial console with 57600 baud to see current radio information and change various settings.
///
/// Wiring
/// ------
/// The necessary wiring of the various chips are described in the Testxxx example sketches.
/// The boards have to be connected by using the following connections:
/// 
/// Arduino port | SI4703 signal | RDA5807FP signal
/// :----------: | :-----------: | :-------------:
/// GND (black)  | GND           | GND   
/// 3.3V (red)   | VCC           | VCC
/// 5V (red)     | -             | -
/// A5 (yellow)  | SCLK          | SCLK
/// A4 (blue)    | SDIO          | SDIO
/// D2           | RST           | -
///
///
/// More documentation and source code is available at http://www.mathertel.de/Arduino
///
/// History:
/// --------
/// * 05.08.2014 created.
/// * 04.10.2014 working.
/// * 20220818 I²S Output Enabled
#include <Wire.h>

#include <radio.h>
#include <RDA5807M.h>
///#include <SI4703.h>
///#include <SI4705.h>
///#include <TEA5767.h>

#include <RDSParser.h>


// Define some stations available at your locations here:
// 89.40 MHz as 8940

RADIO_FREQ preset[] = {
  8770,
  8810, // hr1
  8820,
  8850, // Bayern2
  8890, // ???
  8930, // * hr3
  8980,
  9180,
  9220, 9350,
  9440, // * hr1
  9510, // - Antenne Frankfurt
  9530,
  9560, // Bayern 1
  9680, 9880,
  10020, // planet
  10090, // ffh
  10110, // SWR3
  10030, 10260, 10380, 10400,
  10500 // * FFH
};

int    i_sidx = 5;        ///< Start at Station with index=5

/// The radio object has to be defined by using the class corresponding to the used chip.
/// by uncommenting the right radio object definition.

// RADIO radio;       ///< Create an instance of a non functional radio.
RDA5807M radio;    ///< Create an instance of a RDA5807 chip radio
// SI4703   radio;    ///< Create an instance of a SI4703 chip radio.
//SI4705   radio;    ///< Create an instance of a SI4705 chip radio.
// TEA5767  radio;    ///< Create an instance of a TEA5767 chip radio.


/// get a RDS parser
RDSParser rds;


/// State definition for this radio implementation.
enum RADIO_STATE {
  STATE_PARSECOMMAND, ///< waiting for a new command character.

  STATE_PARSEINT,     ///< waiting for digits for the parameter.
  STATE_EXEC          ///< executing the command.
};

RADIO_STATE state; ///< The state variable is used for parsing input characters.

// - - - - - - - - - - - - - - - - - - - - - - - - - -



/// Update the Frequency on the LCD display.
void DisplayFrequency(RADIO_FREQ f)
{
  char s[12];
  radio.formatFrequency(s, sizeof(s));
  Serial.print("FREQ:"); Serial.println(s);
} // DisplayFrequency()


/// Update the ServiceName text on the LCD display.
void DisplayServiceName(char *name)
{
  Serial.print("RDS:");
  Serial.println(name);
} // DisplayServiceName()


// - - - - - - - - - - - - - - - - - - - - - - - - - -


void RDS_process(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4) {
  rds.processData(block1, block2, block3, block4);
}

/// Execute a command identified by a character and an optional number.
/// See the "?" command for available commands.
/// \param cmd The command character.
/// \param value An optional parameter for the command.
void runSerialCommand(char cmd, int16_t value)
{
  if (cmd == '?') {
    Serial.println();
    Serial.println("? Help");
    Serial.println("+ increase volume");
    Serial.println("- decrease volume");
    Serial.println("> next preset");
    Serial.println("< previous preset");
    Serial.println(". scan up   : scan up to next sender");
    Serial.println(", scan down ; scan down to next sender");
    Serial.println("fnnnnn: direct frequency input");
    Serial.println("i station status");
    Serial.println("s mono/stereo mode");
    Serial.println("b bass boost");
    Serial.println("u mute/unmute");
  }

  // ----- control the volume and audio output -----

  else if (cmd == '+') {
    // increase volume
    int v = radio.getVolume();
    if (v < 15) radio.setVolume(++v);
  } else if (cmd == '-') {
    // decrease volume
    int v = radio.getVolume();
    if (v > 0) radio.setVolume(--v);
  }

  else if (cmd == 'u') {
    // toggle mute mode
    radio.setMute(!radio.getMute());
  }

  // toggle stereo mode
  else if (cmd == 's') { radio.setMono(!radio.getMono()); }

  // toggle bass boost
  else if (cmd == 'b') { radio.setBassBoost(!radio.getBassBoost()); }

  // ----- control the frequency -----

  else if (cmd == '>') {
    // next preset
    if (i_sidx < (sizeof(preset) / sizeof(RADIO_FREQ)) - 1) {
      i_sidx++; radio.setFrequency(preset[i_sidx]);
    } // if
  } else if (cmd == '<') {
    // previous preset
    if (i_sidx > 0) {
      i_sidx--;
      radio.setFrequency(preset[i_sidx]);
    } // if

  } else if (cmd == 'f') { radio.setFrequency(value); }

  else if (cmd == '.') { radio.seekUp(false); } else if (cmd == ':') { radio.seekUp(true); } else if (cmd == ',') { radio.seekDown(false); } else if (cmd == ';') { radio.seekDown(true); }


  // not in help:
  else if (cmd == '!') {
    if (value == 0) radio.term();
    if (value == 1) radio.init();

  } else if (cmd == 'i') {
    char s[12];
    radio.formatFrequency(s, sizeof(s));
    Serial.print("Station:"); Serial.println(s);
    Serial.print("Radio:"); radio.debugRadioInfo();
    Serial.print("Audio:"); radio.debugAudioInfo();

  } // info

  else if (cmd == 'x') {
    radio.debugStatus(); // print chip specific data.
  }
} // runSerialCommand()


/// Setup a FM only radio configuration with I/O for commands and debugging on the Serial port.
void setup() {
  // open the Serial port
  Serial.begin(57600);
  Serial.print("Radio...");
  delay(500);

  // Initialize the Radio 
  radio.init();
/////////////////////////////////////////////////////////////////////////
/////////////REG:04//////////////////////WR//
  Wire.beginTransmission(0x11);
  Wire.write(0x04);
  Wire.write(0b10001000);
//         -[B#FEDCBA98]-  
//             ||||||||______(08)-AFCD AFC disable 0- afc work; 1- afc disabled
//             |||||||_______(09)-SOFTMUTE_EN If 1, softmute enable
//             ||||||________(10)-RDS_FIFO_CLR 1 = clear RDS FIFO //RDSR _MODE???
//             |||||_________(11)-DE De-emphasis 0 = 75 μs; 1 = 50 μs
//             ||||__________(12)-RDS_FIFO_EN 1 = RDS fifo mode enable.
//             |||___________(13)-RBDS 1 = RBDS mode enable 0 = RDS mode only
//             ||____________(14)-STCIEN 1 = IINT ENA : STCIEN = 1 will generate low pulse on GPIO2
//             |_____________(15)-RDSIEN 0 = Disable 1 = Enable Setting STCIEN = 1 will generate a low pulse on GPIO2
  Wire.write(0b01000000);     //  Wire.write(0b01000000);
//         -[B#76543210]-  
//             ||||||||_______(0)-GPIO1[1:0] 00 = High impedance 01 = Reserved
//             |||||||________(1)-GPIO1[1:0] 10 = Low 11 = High
//             ||||||_________(2)-GPIO2[1:0] 00 = High impedance 01 = Interrupt (INT)
//             |||||__________(3)-GPIO2[1:0] 10 = Low 11 = High
//             ||||___________(4)-GPIO3[1:0] 00 = High impedance 01 = Mono/Stereo indicator (ST)
//             |||____________(5)-GPIO3[1:0] 10 = Low 11 = High
//             ||_____________(6)-I2S_ENABLED 1-ENABLED : 0-DISABLED
//             |______________(7)-GPIO1_INT_EN???
  Wire.endTransmission(); // stop transmitting
/////////////REG:04//////////////////////END//
/////////////REG:06//////////////////////WR//
  Wire.beginTransmission(0x011);
  Wire.write(0x06);
  Wire.write(0b00000010);
//         -[B#FEDCBA98]-    
//             ||||||||
//             ||||||||______(08)- WS_I_EDGE 0- normal ws internally 1, inverte ws internally
//             |||||||_______(09)- DATA_SIGNED 0- unsigned 1- signed (16-bit audio data)
//             ||||||________(10)- SCLK_I_EDGE 0- normal sclk 1- inverte sclk internally
//             |||||_________(11)- SW_LR Ws relation to l/r channel 0- ws=0->r, ws=1->l: 1- ws=0->l, ws=1->r
//             ||||__________(12)- I2S_mode_select 0- master mode : 1- slave mode
//             |||___________(13)- OPEN_MODE[1:0]
//             ||____________(14)- OPEN_MODE[1:0]
//             |_____________(15)- RSVD
  Wire.write(0b10000000);
//         -[B#76543210]-    
//             ||||||||
//             ||||||||______(0)-R_DELY-R Channel data Delay 1T
//             |||||||_______(1)-L_DELY-L channel data Delay 1T
//             ||||||________(2)-SCLK_O_EDGE If 1, invert sclk output when as master.
//             |||||_________(3)-SW_O_EDGE If 1, invert ws output when as master.  
//             ||||__________(4)[4]-I2S_SW_CNT[4:0]
//             |||___________(5)[4]-I2S_SW_CNT[4:0]
//             ||____________(6)[4]-I2S_SW_CNT[4:0]
//             |_____________(7)[4]-I2S_SW_CNT[4:0]
//                           4'b1000: WS_STEP=48.000kbps;
//                           4'b0111: WS_STEP=44.100kbps;
//                           4'b0110: WS_STEP=32.000kbps;
//                           4'b0101: WS_STEP=24.000kbps;
//                           4'b0100: WS_STEP=22.050kbps;
//                           4'b0011: WS_STEP=16.000kbps;
//                           4'b0010: WS_STEP=12.000kbps;
//                           4'b0001: WS_STEP=11.025kbps;
//                           4'b0000: WS_STEP=08.000kbps;

  Wire.endTransmission();  // stop transmitting
/////////////REG:06//////////////////////END// 
  // Enable information to the Serial port
  radio.debugEnable();

  radio.setBandFrequency(RADIO_BAND_FM, preset[i_sidx]); // 5. preset.

  // delay(100);

  radio.setMono(false);
  radio.setMute(false);
  // radio._wireDebug();
  radio.setVolume(8);

  Serial.write('>');

  state = STATE_PARSECOMMAND;

  // setup the information chain for RDS data.
  radio.attachReceiveRDS(RDS_process);
  rds.attachServicenNameCallback(DisplayServiceName);

  runSerialCommand('?', 0);
} // Setup


/// Constantly check for serial input commands and trigger command execution.
void loop() {
  int newPos;
  unsigned long now = millis();
  static unsigned long nextFreqTime = 0;
  static unsigned long nextRadioInfoTime = 0;

  // some internal static values for parsing the input
  static char command;
  static int16_t value;
  static RADIO_FREQ lastf = 0;
  RADIO_FREQ f = 0;

  char c;
  if (Serial.available() > 0) {
    // read the next char from input.
    c = Serial.peek();

    if ((state == STATE_PARSECOMMAND) && (c < 0x20)) {
      // ignore unprintable chars
      Serial.read();

    } else if (state == STATE_PARSECOMMAND) {
      // read a command.
      command = Serial.read();
      state = STATE_PARSEINT;

    } else if (state == STATE_PARSEINT) {
      if ((c >= '0') && (c <= '9')) {
        // build up the value.
        c = Serial.read();
        value = (value * 10) + (c - '0');
      } else {
        // not a value -> execute
        runSerialCommand(command, value);
        command = ' ';
        state = STATE_PARSECOMMAND;
        value = 0;
      } // if
    } // if
  } // if


  // check for RDS data
  radio.checkRDS();

  // update the display from time to time
  if (now > nextFreqTime) {
    f = radio.getFrequency();
    if (f != lastf) {
      // print current tuned frequency
      DisplayFrequency(f);
      lastf = f;
    } // if
    nextFreqTime = now + 400;
  } // if  

} // loop

// End.
