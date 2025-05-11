// outil de configuration pour l'imu BWK215S
// écrit par buched
// manu@couvercelle.eu
// dont claim its your job.

#include <EEPROM.h>
// if not in eeprom, overwrite
#define EEP_Ident 2448
//EEPROM
int16_t EEread = 0;
//Variables for settings
struct Storage {
  uint8_t Kb = 1;              // 1 125k 2 250 3 500
};  Storage Settings;      // 11 bytes

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> IMU_Bus;
CAN_message_t msgi;

uint8_t bauds = 0;
uint8_t Service = 0;
float decimal_input;
float unit_input;

void setup(void) {
    Serial.begin(460800);
   EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)            // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, Settings);  
  }
  else
  {
    EEPROM.get(10, Settings);
  }
  IMU_Bus.begin();
  Serial.println("config actuelle : ");
  bauds = Settings.Kb;
  if (bauds == 1)
  {
    IMU_Bus.setBaudRate(125000);
    Serial.println("125k bauds");
    Serial.println("");
  }
  else  if (bauds == 2) 
  {
    IMU_Bus.setBaudRate(250000);
    Serial.println("250k bauds");
    Serial.println("");
  }
  else  if (bauds == 3) 
  {
    IMU_Bus.setBaudRate(500000);
    Serial.println("500k bauds");
    Serial.println("");
  }
  else  if (bauds == 4) 
  {
    IMU_Bus.setBaudRate(1000000);
  }
  Serial.println("Outil de configuration pour Imu BWK215S");
  Serial.println("S pour démarrer");
}

// Décodage de la trame : [0x55 | 0x53 | RollL | RollH | PitchL | PitchH | YawL | YawH]
void decodeFrameCAN(uint8_t *buf) {
  // Extraction valeurs (Little-endian signé)
  int16_t rawX = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t rawY = (int16_t)(buf[5] << 8 | buf[4]);
  int16_t rawZ = (int16_t)(buf[7] << 8 | buf[6]);

  // Conversion selon ta formule
  float angleX = rawX / 32768.0 * 180.0;
  float angleY = rawY / 32768.0 * 180.0;
  float angleZ = rawZ / 32768.0 * 180.0;
  angleZ = -angleZ;
if (angleZ < 0) angleZ += 360.0f;

  int rollx = angleY * 10;
  int pitchx = angleX * 10;
  int yawx = angleZ * 10;
  int roll =rollx;
  int pitch = pitchx;
  int yaw = yawx;

  //Affichage clair des résultats
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("°, Pitch: ");
  Serial.print(pitch);
  Serial.print("°, Yaw: ");
  Serial.print(yaw);
  Serial.println("°");
}

// Fonction pour décoder X et Y (3 octets, BCD)
float decodeAngleXY(const byte *dataIn) {
  float angle = (dataIn[0] & 0x0F) * 100.0
              + (dataIn[1] >> 4) * 10.0
              + (dataIn[1] & 0x0F) * 1.0
              + (dataIn[2] >> 4) * 0.1
              + (dataIn[2] & 0x0F) * 0.01;

  if (dataIn[0] & 0xF0) {
    angle = -angle;
  }

  return angle;
}

float decode_bcd_angle(uint8_t b1, uint8_t b2) {
  int hundreds = (b1 & 0xF0) >> 4;  // bits 4–7
  int tens     = (b1 & 0x0F);       // bits 0–3
  int units    = (b2 & 0xF0) >> 4;  // bits 4–7
  int tenths   = (b2 & 0x0F);       // bits 0–3

  float value = (hundreds * 1000.0) + (tens * 100.0) + (units * 10.0) + (tenths * 1);
    value = -value;
if (value < 0) value += 3600.0f;
  return value;
}


void loop() {

        if (Serial.available())
          {        // Read Data From Serial Monitor 
            byte b = Serial.read();
        
            while (Serial.available()){
              Serial.read();              //Clear the serial buffer
            }
        
            if ( b == 'S') {
              Service = 1; 
              serv();
            }
          }
            if ( IMU_Bus.read(msgi) ) 
            {
              if (msgi.id == 0x585)
                {
        float angleX = decodeAngleXY(&msgi.buf[0]);
        float angleY = decodeAngleXY(&msgi.buf[3]);
        float angleZ = decode_bcd_angle(msgi.buf[6], msgi.buf[7]); // D7-D8
  
        Serial.print("Angle X: ");
        Serial.print(angleX, 2);
        Serial.print(" °, Y: ");
        Serial.print(angleY, 2);
        Serial.print(" °, Z: ");
        Serial.print(angleZ, 2);
        Serial.println(" °");
                  
            }
          if (msgi.id == 0x50)
            {
              decodeFrameCAN(msgi.buf);
            }
         }
}

void serv() {
  Serial.println("\r\nAgOpenGPS CANBUS Service Tool Mode:");
  Help();
   while (Service == 1) 
  { 
      if (Serial.available())   // Read Data From Serail Monitor 
      {    
        byte b = Serial.read();
        if ( b == '?') Help();          
        else if ( b == 'X') exi(); //Exit Service Mode
        else if ( b == 'a') all();
        else if ( b == '0') setoutput();
        else if ( b == '1') vgt();
        else if ( b == '2') cvcqk();
        else if ( b == '3') dcqtk();
        else if ( b == '4') cctk();
        else
        {
          Serial.println("No command, send ? for help");
          Serial.println(" ");
          delay(50);
        }

        while (Serial.available())
        {
        Serial.read();                //Clear the serial buffer
        }
      }
  }
}
 void exi() {
    Serial.println("fin du mode service");
  Service  = 0;
  //save in EEPROM and restart
  EEPROM.put(10, Settings);
  delay(2000);
  //reboo();
  }
void Help() {
  Serial.println("X = service mode exit");  
  Serial.println("a = OneClic config");
  Serial.println("0 = configurer en 3 angles output");
  Serial.println("1 = configurer en 20htz");
  Serial.println("2 = configurer en 125k bauds");
  Serial.println("3 = configurer en 250k bauds");
  Serial.println("4 = configurer en 500k bauds");  
//  Serial.println("");
//  Serial.println("config actuelle : ");
//  Serial.println(CanBauds);
  }

void all() {
  msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x56;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x00;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("IMU réglé sur 3 angles output");
  delay(2000);
  save();
  delay(2000);
  msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x0C;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x03;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("IMU réglé en 20 htz");
  delay(2000);
  save();
  delay(2000);
  msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x20;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x02;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("réglé sur 250k bauds");
  delay(2000);
  save();
  delay(500);
  Serial.println("Patientez, reboot en cours");
  Settings.Kb  = 2;
  //save in EEPROM and restart
  EEPROM.put(10, Settings);
  delay(2000);
  reboo();
  }
  
void setoutput() {
    msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x56;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x00;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("IMU réglé sur 3 angles output");
  delay(2000);
  save();
  }
  
void vgt() {
  msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x0C;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x03;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("IMU réglé en 10 htz");
  delay(2000);
  save();
}

void cvcqk() {
    msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x20;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x03;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("réglé sur 125k bauds");
  delay(2000);
  save();
  delay(500);
  Serial.println("Patientez, reboot en cours");
  Settings.Kb  = 1;
  //save in EEPROM and restart
  EEPROM.put(10, Settings);
  delay(2000);
  reboo();
  }
  
void dcqtk() {
    msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x20;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x02;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("réglé sur 250k bauds");
  delay(2000);
  save();
  delay(500);
  Serial.println("Patientez, reboot en cours");
  Settings.Kb  = 2;
  //save in EEPROM and restart
  EEPROM.put(10, Settings);
  delay(2000);
  reboo();
  }

void cctk() {
    msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x20;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x01;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  Serial.println("réglé sur 500k bauds");
  delay(2000);
  save();
  delay(500);
  Serial.println("Patientez, reboot en cours");
  Settings.Kb  = 3;
  //save in EEPROM and restart
  EEPROM.put(10, Settings);
  delay(2000);
  reboo();
  }
  
void save() {
  msgi.id = 0x605;
  msgi.flags.extended = false;
  msgi.len = 8;
  msgi.buf[0] = 0x40;
  msgi.buf[1] = 0x0A;
  msgi.buf[2] = 0x10;
  msgi.buf[3] = 0x00;
  msgi.buf[4] = 0x00;
  msgi.buf[5] = 0x00;
  msgi.buf[6] = 0x00;
  msgi.buf[7] = 0x00;
  IMU_Bus.write(msgi);
  
  Serial.println("saved");
}

void reboo() {
  SCB_AIRCR = 0x05FA0004; //Teensy Reset
             }
