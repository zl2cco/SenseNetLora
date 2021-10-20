#include <pga460.h>

// Define UART commands by name
// Single Address
byte P1BL  = 0x00;
byte P2BL  = 0x01;
byte P1LO  = 0x02;
byte P2LO  = 0x03;
byte TNLM  = 0x04;
byte UMR   = 0x05;
byte TNLR  = 0x06;
byte TEDD  = 0x07;
byte SDIAG = 0x08;
byte SRR   = 0x09;
byte SRW   = 0x0A;
byte EEBR  = 0x0B;
byte EEBW  = 0x0C;
byte TVGBR = 0x0D;
byte TVGBW = 0x0E;
byte THRBR = 0x0F;
byte THRBW = 0x10;
//Broadcast
byte BC_P1BL = 0x11;
byte BC_P2BL = 0x12;
byte BC_P1LO = 0x13;
byte BC_P2LO = 0x14;
byte BC_TNLM = 0x15;
byte BC_RW = 0x16;
byte BC_EEBW = 0x17;
byte BC_TVGBW = 0x18;
byte BC_THRBW = 0x19;
//CMDs 26-31 are reserved


// List user registers by name with default settings from TI factory
byte USER_DATA1 = 0x00;
byte USER_DATA2 = 0x00;
byte USER_DATA3 = 0x00;
byte USER_DATA4 = 0x00;
byte USER_DATA5 = 0x00;
byte USER_DATA6 = 0x00;
byte USER_DATA7 = 0x00;
byte USER_DATA8 = 0x00;
byte USER_DATA9 = 0x00;
byte USER_DATA10 = 0x00;
byte USER_DATA11 = 0x00;
byte USER_DATA12 = 0x00;
byte USER_DATA13 = 0x00;
byte USER_DATA14 = 0x00;
byte USER_DATA15 = 0x00;
byte USER_DATA16 = 0x00;
byte USER_DATA17 = 0x00;
byte USER_DATA18 = 0x00;
byte USER_DATA19 = 0x00;
byte USER_DATA20 = 0x00;
byte TVGAIN0 = 0xAF;
byte TVGAIN1 = 0xFF;
byte TVGAIN2 = 0xFF;
byte TVGAIN3 = 0x2D;
byte TVGAIN4 = 0x68;
byte TVGAIN5 = 0x36;
byte TVGAIN6 = 0xFC;
byte INIT_GAIN = 0xC0;
byte FREQUENCY = 0x8C;
byte DEADTIME = 0x00;
byte PULSE_P1 = 0x01;
byte PULSE_P2 = 0x12;
byte CURR_LIM_P1 = 0x47;
byte CURR_LIM_P2 = 0xFF;
byte REC_LENGTH = 0x1C;
byte FREQ_DIAG = 0x00;
byte SAT_FDIAG_TH = 0xEE;
byte FVOLT_DEC = 0x7C;
byte DECPL_TEMP = 0x0A;
byte DSP_SCALE = 0x00;
byte TEMP_TRIM = 0x00;
byte P1_GAIN_CTRL = 0x00;
byte P2_GAIN_CTRL = 0x00;
byte EE_CRC = 0xFF;
byte EE_CNTRL = 0x00;

byte P1_THR_0 = 0x98;
//byte P1_THR_0 = 0x88;
byte P1_THR_1 = 0x88;
byte P1_THR_2 = 0x88;
byte P1_THR_3 = 0x88;
byte P1_THR_4 = 0x88;
byte P1_THR_5 = 0x88;
byte P1_THR_6 = 0xFC;
// byte P1_THR_6 = 0x84;
byte P1_THR_7 = 0x21;
byte P1_THR_8 = 0x08;
byte P1_THR_9 = 0x42;
byte P1_THR_10 = 0x10;

byte P1_THR_11 = 0x80;
byte P1_THR_12 = 0x80;
byte P1_THR_13 = 0x80;
byte P1_THR_14 = 0x80;
byte P1_THR_15 = 0x07;

byte P2_THR_0 = 0x88;
byte P2_THR_1 = 0x88;
byte P2_THR_2 = 0x88;
byte P2_THR_3 = 0x88;
byte P2_THR_4 = 0x88;
byte P2_THR_5 = 0x88;
byte P2_THR_6 = 0x84;
byte P2_THR_7 = 0x21;
byte P2_THR_8 = 0x08;
byte P2_THR_9 = 0x42;
byte P2_THR_10 = 0x10;
byte P2_THR_11 = 0x80;
byte P2_THR_12 = 0x80;
byte P2_THR_13 = 0x80;
byte P2_THR_14 = 0x80;
byte P2_THR_15 = 0x80;

PGA460::PGA460(int en_pin, int tst_pin, Uart* serial_port)
{
  _en_pin = en_pin;
  _tst_pin = tst_pin;
  _serial_port = serial_port;
  status = 0;
}


void PGA460::begin()
{
  // Disable power supply
  pinMode(_en_pin, OUTPUT);
  digitalWrite(_en_pin, LOW);

  // Select 3.3V digital voltage level
  pinMode(_tst_pin, OUTPUT);
  digitalWrite(_tst_pin, LOW);
  
  // Enable the power supply
  digitalWrite(_en_pin, HIGH);
  delay(100);
  pinMode(_tst_pin, INPUT_PULLUP);

  _serial_port->begin(57600, SERIAL_8N2);

  delay(100);

  // Configure PGA460
  write_default_values();   // load default values

  set_thresholds(0);        // Set thresholds
//  set_thresholds(4);        // Set thresholds

  writereg(0x1C, 52);       // Frequency
  writereg(0x1E, 0x03);     // Preset 1 sends 3 pulses; and UART_DIAG UART diagnostic page

//   writereg(0x22, 0x22);     // Set record time to 4.18m (12.288ms)
//   writereg(0x1E, 0x21);     // Preset 1 sends 3 pulses; and UART_DIAG UART diagnostic page
//   //writereg(0x1E, 0x63);     // Preset 1 sends 3 pulses; and System diagnostic page
//   //writereg(0x1B, 0xC0);       // bandwidth 
//   writereg(0x1D, 0x10);       // Set threshold level comparator deglitch time
//   writereg(0x20, 0x3F);       // Enable current limit
// //  writereg(0x26, 0x42);       // time decouple, AFE_GAIN_RNG=52-84dB
//   writereg(0x26, 0x02);       // time decouple, AFE_GAIN_RNG=58-90dB

//   set_tv_gain(3);


  delay(10);


  // Check status
  uint8_t diag, data;
  status=0;

  readreg(0x4C, &diag, &data);
  SerialUSB.println("DEV_STAT0: " + String(data));
  readreg(0x4D, &diag, &data);
  SerialUSB.println("DEV_STAT1: " + String(data));

  if (readreg(0x1C, &diag, &data)) {
    SerialUSB.println("Read FREQUENCY register: " + String(data) + " DIAG: " + String(diag));
    if (data != 52) status++;
  }
  else status++;
  if (readreg(0x1E, &diag, &data)) {
    SerialUSB.println("Read PULSE_P1 register: " + String(data) + " DIAG: " + String(diag));
    if (data != 0x63) status++;
  }
  else status++;
}

void PGA460::end()
{
  // writereg(0x25, FVOLT_DEC || 0x18);
  // writereg(0x26, DECPL_TEMP || 0x20);
  
  deep_sleep();
  _serial_port->end();
}

void PGA460::write_default_values()
{
    // List user registers by name with default settings from TI factory
  writereg(0x00 ,USER_DATA1) ;
  writereg(0x01 ,USER_DATA2) ;
  writereg(0x02 ,USER_DATA3) ;
  writereg(0x03 ,USER_DATA4) ;
  writereg(0x04 ,USER_DATA5) ;
  writereg(0x05 ,USER_DATA6) ;
  writereg(0x06 ,USER_DATA7) ;
  writereg(0x07 ,USER_DATA8) ;
  writereg(0x08 ,USER_DATA9) ;
  writereg(0x09 ,USER_DATA10) ;
  writereg(0x0A ,USER_DATA11) ;
  writereg(0x0B ,USER_DATA12) ;
  writereg(0x0C ,USER_DATA13) ;
  writereg(0x0D ,USER_DATA14) ;
  writereg(0x0E ,USER_DATA15) ;
  writereg(0x0F ,USER_DATA16) ;
  writereg(0x10 ,USER_DATA17) ;
  writereg(0x11 ,USER_DATA18) ;
  writereg(0x12 ,USER_DATA19) ;
  writereg(0x13 ,USER_DATA20) ;
  writereg(0x14 ,TVGAIN0) ;
  writereg(0x15 ,TVGAIN1) ;
  writereg(0x16 ,TVGAIN2) ;
  writereg(0x17 ,TVGAIN3) ;
  writereg(0x18 ,TVGAIN4) ;
  writereg(0x19 ,TVGAIN5) ;
  writereg(0x1A ,TVGAIN6) ;
  writereg(0x1B ,INIT_GAIN) ;
  writereg(0x1C ,FREQUENCY) ;
  writereg(0x1D ,DEADTIME) ;
  writereg(0x1E ,PULSE_P1) ;
  writereg(0x1F ,PULSE_P2) ;
  writereg(0x20 ,CURR_LIM_P1) ;
  writereg(0x21 ,CURR_LIM_P2) ;
  writereg(0x22 ,REC_LENGTH) ;
  writereg(0x23 ,FREQ_DIAG) ;
  writereg(0x24 ,SAT_FDIAG_TH) ;
  writereg(0x25 ,FVOLT_DEC) ;
  writereg(0x26 ,DECPL_TEMP) ;
  writereg(0x27 ,DSP_SCALE) ;
  writereg(0x28 ,TEMP_TRIM) ;
  writereg(0x29 ,P1_GAIN_CTRL) ;
  writereg(0x2A ,P2_GAIN_CTRL) ;
  writereg(0x2B ,EE_CRC) ;
  writereg(0x40 ,EE_CNTRL) ;
  writereg(0x5F ,P1_THR_0) ;
  writereg(0x60 ,P1_THR_1) ;
  writereg(0x61 ,P1_THR_2) ;
  writereg(0x62 ,P1_THR_3) ;
  writereg(0x63 ,P1_THR_4) ;
  writereg(0x64 ,P1_THR_5) ;
  writereg(0x65 ,P1_THR_6) ;
  writereg(0x66 ,P1_THR_7) ;
  writereg(0x67 ,P1_THR_8) ;
  writereg(0x68 ,P1_THR_9) ;
  writereg(0x69 ,P1_THR_10) ;
  writereg(0x6A ,P1_THR_11) ;
  writereg(0x6B ,P1_THR_12) ;
  writereg(0x6C ,P1_THR_13) ;
  writereg(0x6D ,P1_THR_14) ;
  writereg(0x6E ,P1_THR_15) ;
  writereg(0x6F ,P2_THR_0) ;
  writereg(0x70 ,P2_THR_1) ;
  writereg(0x71 ,P2_THR_2) ;
  writereg(0x72 ,P2_THR_3) ;
  writereg(0x73 ,P2_THR_4) ;
  writereg(0x74 ,P2_THR_5) ;
  writereg(0x75 ,P2_THR_6) ;
  writereg(0x76 ,P2_THR_7) ;
  writereg(0x77 ,P2_THR_8) ;
  writereg(0x78 ,P2_THR_9) ;
  writereg(0x79 ,P2_THR_10) ;
  writereg(0x7A ,P2_THR_11) ;
  writereg(0x7B ,P2_THR_12) ;
  writereg(0x7C ,P2_THR_13) ;
  writereg(0x7D ,P2_THR_14) ;
  writereg(0x7E ,P2_THR_15) ;
}

void PGA460::deep_sleep()
{
  digitalWrite(_en_pin, LOW);
}

int PGA460::get_status()
{
  return status;
}

uint8_t PGA460::calc_checksum(uint8_t* buf, int len) {
  uint16_t sum = 0;
  uint8_t carry = 0;

  for (int i = 0; i < len; i++) {
    sum = sum + buf[i] + carry;
    if (i > 0)
      carry = (sum >> 8) & 0x01;
    sum = sum & 0xff;
  }

  sum = (sum + carry) & 0xff;
  sum = (~sum) & 0xff;
  return (uint8_t)sum;
}


bool PGA460::readreg(uint8_t regaddr, uint8_t* diag, uint8_t* regdata) {

  uint8_t buf[4] = {0x55, 0x09, 0x1B, 0xDB};
  uint8_t len = 4;

  // C-TO-P
  buf[0]   = 0x55;
  buf[1]   = SRR;
  buf[2]   = regaddr;
  buf[len - 1] = calc_checksum(&buf[1], len - 2);

   _serial_port->write(buf, len);

  // P-TO-C
   _serial_port->readBytes(buf, 3);

  *diag = buf[0];
  *regdata = buf[1];

  if (!(*diag & 0x40)) SerialUSB.println(*diag, HEX);

  if ((*diag & 0x40) && calc_checksum(buf, 2) == buf[2])
    return true;
  else
    return false;
}

void PGA460::writereg(uint8_t reg, uint8_t data) {
  uint8_t buf[5];
  uint8_t len = 5;

  buf[0]   = 0x55;
  buf[1]   = 0x0A;
  buf[2]   = reg;
  buf[3]   = data;
  buf[4]   = calc_checksum(&buf[1], len - 2);

   _serial_port->write(buf, len);
  delay(1);

}

void PGA460::set_thresholds(byte thr)
{
  byte buf[50];

  buf[0] = 0x55;
  buf[1] = 16;

  switch (thr)
  {
    case 0: //25% Levels 64 of 255
//      buf[2] = P1_THR_0 = 0x98;
      buf[2] = P1_THR_0 = 0x77;
      buf[3] = P1_THR_1 = 0x88;
      buf[4] = P1_THR_2 = 0x88;
      buf[5] = P1_THR_3 = 0x88;
      buf[6] = P1_THR_4 = 0x88;
      buf[7] = P1_THR_5 = 0x88;
      buf[8] = P1_THR_6   = 0xFF;
//      buf[8] = P1_THR_6   = 0x21;
      buf[9] = P1_THR_7   = 0xC8;
//      buf[9] = P1_THR_7   = 0x08;
      buf[10] = P1_THR_8  = 0x42;
      buf[11] = P1_THR_9  = 0x10;
      buf[12] = P1_THR_10 = 0x84;
      buf[13] = P1_THR_11 = 0x26;
      buf[14] = P1_THR_12 = 0x26;
      buf[15] = P1_THR_13 = 0x26;
      buf[16] = P1_THR_14 = 0x26;
      buf[17] = P1_THR_15 = 0x07;
//      buf[17] = P1_THR_15 = 0x00;

      buf[18] = P2_THR_0 = 0x88;
      buf[19] = P2_THR_1 = 0x88;
      buf[20] = P2_THR_2 = 0x88;
      buf[21] = P2_THR_3 = 0x88;
      buf[22] = P2_THR_4 = 0x88;
      buf[23] = P2_THR_5 = 0x88;
      buf[24] = P2_THR_6 = 0x42;
      buf[25] = P2_THR_7 = 0x10;
      buf[26] = P2_THR_8 = 0x84;
      buf[27] = P2_THR_9 = 0x21;
      buf[28] = P2_THR_10 = 0x08;
      buf[29] = P2_THR_11 = 0x40;
      buf[30] = P2_THR_12 = 0x40;
      buf[31] = P2_THR_13 = 0x40;
      buf[32] = P2_THR_14 = 0x40;
      buf[33] = P2_THR_15 = 0x00;
      break;
    case 1: //50% Level (midcode) 128 of 255
      buf[2] = P1_THR_0 = 0x98;
      buf[3] = P1_THR_1 = 0x88;
      buf[4] = P1_THR_2 = 0x88;
      buf[5] = P1_THR_3 = 0x88;
      buf[6] = P1_THR_4 = 0x88;
      buf[7] = P1_THR_5 = 0x88;
      buf[8] = P1_THR_6 = 0x84;
      buf[9] = P1_THR_7 = 0x21;
      buf[10] = P1_THR_8 = 0x42;
      buf[11] = P1_THR_9 = 0x10;
      buf[12] = P1_THR_10 = 0x10;
      buf[13] = P1_THR_11 = 0x80;
      buf[14] = P1_THR_12 = 0x80;
      buf[15] = P1_THR_13 = 0x80;
      buf[16] = P1_THR_14 = 0x80;
      buf[17] = P1_THR_15 = 0x00;
      buf[18] = P2_THR_0 = 0x88;
      buf[19] = P2_THR_1 = 0x88;
      buf[20] = P2_THR_2 = 0x88;
      buf[21] = P2_THR_3 = 0x88;
      buf[22] = P2_THR_4 = 0x88;
      buf[23] = P2_THR_5 = 0x88;
      buf[24] = P2_THR_6 = 0x84;
      buf[25] = P2_THR_7 = 0x21;
      buf[26] = P2_THR_8 = 0x42;
      buf[27] = P2_THR_9 = 0x10;
      buf[28] = P2_THR_10 = 0x10;
      buf[29] = P2_THR_11 = 0x80;
      buf[30] = P2_THR_12 = 0x80;
      buf[31] = P2_THR_13 = 0x80;
      buf[32] = P2_THR_14 = 0x80;
      buf[33] = P2_THR_15 = 0x00;
      break;
    case 2: //75% Levels 192 of 255
      buf[2] = P1_THR_0 = 0x98;
      buf[3] = P1_THR_1 = 0x88;
      buf[4] = P1_THR_2 = 0x88;
      buf[5] = P1_THR_3 = 0x88;
      buf[6] = P1_THR_4 = 0x88;
      buf[7] = P1_THR_5 = 0x88;
      buf[8] = P1_THR_6 = 0xC6;
      buf[9] = P1_THR_7 = 0x31;
      buf[10] = P1_THR_8 = 0x8C;
      buf[11] = P1_THR_9 = 0x63;
      buf[12] = P1_THR_10 = 0x18;
      buf[13] = P1_THR_11 = 0xC0;
      buf[14] = P1_THR_12 = 0xC0;
      buf[15] = P1_THR_13 = 0xC0;
      buf[16] = P1_THR_14 = 0xC0;
      buf[17] = P1_THR_15 = 0x00;
      buf[18] = P2_THR_0 = 0x88;
      buf[19] = P2_THR_1 = 0x88;
      buf[20] = P2_THR_2 = 0x88;
      buf[21] = P2_THR_3 = 0x88;
      buf[22] = P2_THR_4 = 0x88;
      buf[23] = P2_THR_5 = 0x88;
      buf[24] = P2_THR_6 = 0xC6;
      buf[25] = P2_THR_7 = 0x31;
      buf[26] = P2_THR_8 = 0x8C;
      buf[27] = P2_THR_9 = 0x63;
      buf[28] = P2_THR_10 = 0x18;
      buf[29] = P2_THR_11 = 0xC0;
      buf[30] = P2_THR_12 = 0xC0;
      buf[31] = P2_THR_13 = 0xC0;
      buf[32] = P2_THR_14 = 0xC0;
      buf[33] = P2_THR_15 = 0x00;
      break;

    case 3: // BASED ON ECHO DUMP
      buf[2] = P1_THR_0 = 0x98;
      buf[3] = P1_THR_1 = 0x88;
      buf[4] = P1_THR_2 = 0x88;
      buf[5] = P1_THR_3 = 0x88;
      buf[6] = P1_THR_4 = 0x88;
      buf[7] = P1_THR_5 = 0x88;
      buf[8] = P1_THR_6 = 0xf8;
      buf[9] = P1_THR_7 = 0x84;
      buf[10] = P1_THR_8 = 0x21;
      buf[11] = P1_THR_9 = 0x8;
      buf[12] = P1_THR_10 = 0x42;
      buf[13] = P1_THR_11 = 0x14;
      buf[14] = P1_THR_12 = 0x14;
      buf[15] = P1_THR_13 = 0x14;
      buf[16] = P1_THR_14 = 0x14;

      buf[17] = P1_THR_15 = 7;

      buf[18] = P2_THR_0 = 0x88;
      buf[19] = P2_THR_1 = 0x88;
      buf[20] = P2_THR_2 = 0x88;
      buf[21] = P2_THR_3 = 0x88;
      buf[22] = P2_THR_4 = 0x88;
      buf[23] = P2_THR_5 = 0x88;
      buf[24] = P2_THR_6 = 0x42;
      buf[25] = P2_THR_7 = 0x10;
      buf[26] = P2_THR_8 = 0x84;
      buf[27] = P2_THR_9 = 0x21;
      buf[28] = P2_THR_10 = 0x08;
      buf[29] = P2_THR_11 = 0x40;
      buf[30] = P2_THR_12 = 0x40;
      buf[31] = P2_THR_13 = 0x40;
      buf[32] = P2_THR_14 = 0x40;
      buf[33] = P2_THR_15 = 0x00;
      
      break;

    case 4:
      buf[2] = P1_THR_0 = 0x98;
      buf[3] = P1_THR_1 = 0x88;
      buf[4] = P1_THR_2 = 0x88;
      buf[5] = P1_THR_3 = 0x88;
      buf[6] = P1_THR_4 = 0x88;
      buf[7] = P1_THR_5 = 0x88;
      buf[8] = P1_THR_6 = 0xf9;
      buf[9] = P1_THR_7 = 0x8c;
      buf[10] = P1_THR_8 = 0x63;
      buf[11] = P1_THR_9 = 0x18;
      buf[12] = P1_THR_10 = 0xc6;
      buf[13] = P1_THR_11 = 0x32;
      buf[14] = P1_THR_12 = 0x32;
      buf[15] = P1_THR_13 = 0x32;
      buf[16] = P1_THR_14 = 0x32;
      buf[17] = P1_THR_15 = 7;

      buf[18] = P2_THR_0 = 0x88;
      buf[19] = P2_THR_1 = 0x88;
      buf[20] = P2_THR_2 = 0x88;
      buf[21] = P2_THR_3 = 0x88;
      buf[22] = P2_THR_4 = 0x88;
      buf[23] = P2_THR_5 = 0x88;
      buf[24] = P2_THR_6 = 0x42;
      buf[25] = P2_THR_7 = 0x10;
      buf[26] = P2_THR_8 = 0x84;
      buf[27] = P2_THR_9 = 0x21;
      buf[28] = P2_THR_10 = 0x08;
      buf[29] = P2_THR_11 = 0x40;
      buf[30] = P2_THR_12 = 0x40;
      buf[31] = P2_THR_13 = 0x40;
      buf[32] = P2_THR_14 = 0x40;
      buf[33] = P2_THR_15 = 0x00;
      break;

    default: break;
  }

  buf[34] = calc_checksum(&buf[1], 33);
  _serial_port->write(buf, 35);
  delay(1);
}



void PGA460::set_tv_gain(uint8_t dB) {
  writereg(0x1B, 0);                // INIT_GAIN to default value
  writereg(0x14, 0x98);             // TVG_T0/TVG_T1 1400us
  writereg(0x15, 0x88);             // TVG_T2/TVG_T3 1400us
  writereg(0x16, 0x88);             // TVG_T4/TVG_T5 1400us
  writereg(0x17, 0x00);             // TVG_G1/TVG_G2 +0.5dB
  writereg(0x18, (dB<<4) | (dB));   // TVG_G2/TVG_G3 +3dB
  writereg(0x19, (dB<<4) | (dB));   // TVG_G3/TVG_G4 +3dB
  writereg(0x1A, dB<<2);            // TVG_G5        +3dB    
}


bool PGA460::getUSmeasurement(uint8_t preset, uint16_t *tof, uint8_t *width, uint8_t* peak_amp, uint8_t *diag) {
  uint8_t buf[6] = {0x55, 0x00, 0x00, 0x00, 0x00};
  uint8_t len;
  uint8_t checksum;
  bool    ret;

  // Burst and listen
  len        = 4;
  buf[1]     = (preset == 1) ? 0 : 1;
  buf[2]     = 1;
  checksum   = calc_checksum(&buf[1], len - 2);
  buf[len - 1] = checksum;
  _serial_port->write(buf, len);
  delay(20);


  //Get ultrasonic measurement
  buf[1]     = 5;
  len        = 2;
  // checksum   = calc_checksum(&buf[1], len - 2);
  // buf[len - 1] = checksum;
  _serial_port->write(buf, len);

  // SerialUSB.print("RX <--- ");
  len = 6;
  _serial_port->readBytes(buf, len);

  checksum = calc_checksum(buf, len - 1);
  if (checksum == buf[len - 1])
    ret = true;
  else
    ret = false;

  if (buf[0] != 0x40) {
    ret = false;
  }

  *diag     = buf[0];
  *tof      = (uint16_t)buf[1];
  *tof      = (*tof << 8);
  *tof      = *tof + buf[2];
  *width    = buf[3];
  *peak_amp = buf[4];

    return(ret);
}


double PGA460::get_distance(obj_t *obj) {
  uint16_t tof      = 0;
  uint8_t  width    = 0;
  uint8_t  peak_amp = 0;
  double   dist_m   = 0.0;
  double   dtof     = 0.0;
  uint8_t  diag     = 0;
  int      count    = 0;
  uint8_t  errors   = 0;

  // Disable echo data dump
  writereg(0x40, 0x00); // DATADUMP_EN=0 in EE_CNTRL

    for (int i=0; i<1; i++) {
        if (getUSmeasurement(1, &tof, &width, &peak_amp, &diag)) {
            count++;
            dtof = (double)tof;
            dtof = dtof/1000000.0;
            dtof = dtof/2.0;
            dist_m += (343.0 * dtof ) - 0.01715 + (343.0 * (4.0/48000.0) / 2 );
        }
        else
            errors++;
    }

    if (count > 0)
        dist_m = dist_m/count;

    obj->distance_m = dist_m;

    return(dist_m);

}




bool PGA460::getNOBALdata(uint8_t *data, uint8_t *diag) {
  uint8_t  buf[130] = {0x55, 0x00, 0x00, 0x00, 0x00};
  uint16_t echodata[128];
  uint8_t  len;
  uint8_t  checksum;
  bool     ret;
  int      i;

  // Initialise echodata array
  for (i=0; i<128;i++) {
    echodata[i] = 0;
  }

  // Enable echo data dump
  writereg(0x40, 0x80); // DATADUMP_EN=1 in EE_CNTRL

  SerialUSB.println("Taking 100 measurements: ");

  // Take 100 measurements
  for (i=0; i<100; i++) {
    SerialUSB.print(".");
    // Burst and listen
    len        = 4;
    buf[1]     = 0;
    buf[2]     = 1;
    checksum   = calc_checksum(&buf[1], len - 2);
    buf[len - 1] = checksum;
    _serial_port->write(buf, len);
    delay(20);


    //Get transducer echo data dump
    buf[1]     = 7;
    len        = 2;
    //checksum   = calc_checksum(&buf[1], len - 2);
    //buf[len - 1] = checksum;
    _serial_port->write(buf, len);

    len = 2 + 128;
    _serial_port->readBytes(buf, len);

    checksum = calc_checksum(buf, len - 1);
    if (checksum == buf[len - 1])
      ret = true;
    else
      ret = false;

    if (buf[0] != 0x40) {
      ret = false;
    }

    for (int i=1; i<129; i++) {
      echodata[i-1] = echodata[i-1]  +  buf[i];
    }

    *diag     = buf[0];

    delay(20);

  }


  // Calculate average echo data, save data and print result  
  SerialUSB.println("Echo Data: ");
  for (i=0; i<128; i++) {
    echodata[i] = echodata[i] / 100;
    SerialUSB.println(echodata[i]);
    data[i] = echodata[i];
  }

  // Disable echo data dump
  writereg(0x40, 0x00); // DATADUMP_EN=0 in EE_CNTRL


    return(ret);
}