#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

float decimal_input;
float unit_input;

uint8_t KeyaSteerPGN[] = { 0x23, 0x00, 0x20, 0x01, 0,0,0,0 }; // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = { 0, 0, 0, 0, 0, 0, 0, 0, };

// templates for matching responses of interest
uint8_t keyaCurrentResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint64_t KeyaPGN = 0x06000001;

const bool debugKeya = true;

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
  return value;
}

void keyaSend(uint8_t data[]) {
  CAN_message_t KeyaBusSendData;
KeyaBusSendData.id = KeyaPGN;
KeyaBusSendData.flags.extended = true;
KeyaBusSendData.len = 8;  // ou autre
memcpy(KeyaBusSendData.buf, data, KeyaBusSendData.len);
Keya_Bus.write(KeyaBusSendData);
//  CAN_message_t KeyaBusSendData;
//  KeyaBusSendData.id = KeyaPGN;
//  KeyaBusSendData.flags.extended = true;
//  KeyaBusSendData.len = 8;
//  memcpy(KeyaBusSendData.buf, data, sizeof(data));
//  Keya_Bus.write(KeyaBusSendData);
}

void CAN_Setup() {
  I_Bus.begin();
  I_Bus.setBaudRate(250000);
  Keya_Bus.begin();
  Keya_Bus.setBaudRate(250000);
  K_Bus.begin();
  if (Brand == 5) K_Bus.setBaudRate(500000);
  else K_Bus.setBaudRate(250000);
  K_Bus.enableFIFO();
  K_Bus.setFIFOFilter(REJECT_ALL);
    switch (Brand)
    {
      case 0:
          K_Bus.setFIFOFilter(0, 0x18EF1CD2, EXT);  //Claas Engage Message
          K_Bus.setFIFOFilter(1, 0x1CFFE6D2, EXT);  //Claas Work Message (CEBIS Screen MR Models)
          break;
      case 1:
          K_Bus.setFIFOFilter(0, 0x18EF1C32, EXT);  //Valtra Engage Message
          K_Bus.setFIFOFilter(1, 0x18EF1CFC, EXT);  //Mccormick Engage Message
          K_Bus.setFIFOFilter(2, 0x18EF1C00, EXT);  //MF Engage Message
        break;
      case 2:
          K_Bus.setFIFOFilter(0, 0x14FF7706, EXT);  //CaseIH Engage Message
          K_Bus.setFIFOFilter(1, 0x18FE4523, EXT);  //CaseIH Rear Hitch Infomation
          K_Bus.setFIFOFilter(2, 0x18FF1A03, EXT);  //CaseIH Engage Message
        break;
      case 3:
          K_Bus.setFIFOFilter(0, 0x613, STD);  //Fendt Engage
        break;
      case 4:
          K_Bus.setFIFOFilter(0, 0x18EFAB27, EXT);  //JCB engage message
        break;
      case 5:
          K_Bus.setFIFOFilter(0, 0xCFFD899, EXT);  //FendtOne Engage
        break;
      default:
        Serial.println("No brand selected");
        break;
    }
  delay(1000);
  if (debugKeya) Serial.println("Initialised Keya CANBUS");
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
  return memcmp(message.buf, pattern, patternSize) == 0;
}

void disableKeyaSteer() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0c;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
}

void disableKeyaSteerTEST() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x03;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x11;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
}

void enableKeyaSteer() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  if (debugKeya) Serial.println("Enabled Keya motor");
}

void SteerKeya(int steerSpeed) {
  int actualSpeed = map(steerSpeed, -255, 255, -995, 998);
  if (pwmDrive == 0) {
    disableKeyaSteer();
  }
  if (debugKeya) Serial.println("told to steer, with " + String(steerSpeed) + " so....");
  if (debugKeya) Serial.println("I converted that to speed " + String(actualSpeed));

  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x23;
  KeyaBusSendData.buf[1] = 0x00;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x01;
  if (steerSpeed < 0) {
    KeyaBusSendData.buf[4] = highByte(actualSpeed); // TODO take PWM in instead for speed (this is -1000)
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0xff;
    KeyaBusSendData.buf[7] = 0xff;
    if (debugKeya) Serial.println("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
  }
  else {
    KeyaBusSendData.buf[4] = highByte(actualSpeed);
    KeyaBusSendData.buf[5] = lowByte(actualSpeed);
    KeyaBusSendData.buf[6] = 0x00;
    KeyaBusSendData.buf[7] = 0x00;
    if (debugKeya) Serial.println("pwmDrive > zero - anticlock-clockwise - steerSpeed " + String(steerSpeed));
  }
  Keya_Bus.write(KeyaBusSendData);
  enableKeyaSteer();
}


void KeyaBus_Receive()
{
  CAN_message_t KeyaBusReceiveData;
  if (Keya_Bus.read(KeyaBusReceiveData))
      {
        if (KeyaBusReceiveData.id == 0x07000001)
          {
            if (KeyaBusReceiveData.buf[4] == 0xFF)
              {
                KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading  ) + ( 0.05 *  (256 - KeyaBusReceiveData.buf[5]) * 20);
              }
            else 
              {
                KeyaCurrentSensorReading = (0.95 * KeyaCurrentSensorReading  ) + ( 0.05 * KeyaBusReceiveData.buf[5] * 20);
              }
          }
      }
     
  CAN_message_t KBusReceiveData;
  if (K_Bus.read(KBusReceiveData))
    {
      if (Brand == 1)
        {
              if (KBusReceiveData.id == 0x18EF1C32)
              {
                  if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[2])== 1)
                  {
                    eng();
                  }
              } 
  
              if (KBusReceiveData.id == 0x18EF1CFC)//Mccormick engage message
              {
                  if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[3])== 255)
                  {   
                    eng();
                  }
              } 
              if (KBusReceiveData.id == 0x18EF1C00)//MF engage message
              {
                  if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[2])== 1)
                  {   
                    eng();
                  }
              }
          
        }
      if (Brand == 2)
        {
          if (KBusReceiveData.id == 0x14FF7706)   // **edit ID and conditions
            {
              if ((KBusReceiveData.buf[0]) == 130 && (KBusReceiveData.buf[1]) == 1)
                {
                  eng();
                }
              if ((KBusReceiveData.buf[0])== 178 && (KBusReceiveData.buf[1])== 4)
                {
                  eng();
                }
            }
           if (aogConfig.isRelayActiveHigh == 1)
                {
                  if (KBusReceiveData.id == 0x18FE4523)
                    {
                      KBUSRearHitch = (KBusReceiveData.buf[0]);
                      if (KBUSRearHitch < aogConfig.user1) workCAN = 1;
                      else workCAN = 0;
                    }   
                }
        }
      if (Brand == 3)
        {
          if (KBusReceiveData.id == 0x613)
            {
              if (KBusReceiveData.buf[0]==0x15 && KBusReceiveData.buf[2]==0x06 && KBusReceiveData.buf[3]==0xCA)
                {
                  if (KBusReceiveData.buf[1]==0x88 && KBusReceiveData.buf[4]==0x80) // Fendt Auto Steer Go   
                    {
                      eng();
                    }
                }  
            }
        }
     if (Brand == 4)
        {
            if (KBusReceiveData.id == 0x18EFAB27)
            {
                if ((KBusReceiveData.buf[0])== 15 && (KBusReceiveData.buf[1])== 96 && (KBusReceiveData.buf[2])== 1)
                {
                  eng();
                }
            }    
   
        }
      if (Brand == 5)
      {
          if (KBusReceiveData.id == 0xCFFD899)   //**FendtOne Engage Message**  
          {
            if ((KBusReceiveData.buf[3])== 0xF6)
            {   
                eng();
            }
          }
      }
    }
}

void I_receive()
{
  CAN_message_t msgim;
  if (I_Bus.read(msgim)) 
     {
        if (msgim.id == 0x585)
          {
              roll = decodeAngleXY(&msgim.buf[0]);
              pitch = decodeAngleXY(&msgim.buf[3]);                  
              yaw = 3600 - decode_bcd_angle(msgim.buf[6], msgim.buf[7]); // D7-D8

              roll *= 10.0;
              pitch *= 10.0;
              
              if(steerConfig.InvertWAS)
                {
                  roll *= -1;
                }
    
              // Échange Roll et Pitch selon la variable swapRollPitch
              if (steerConfig.IsUseY_Axis)
                {
                  float temp = roll;
                  roll = pitch;
                  pitch = temp;
                }
         }
      if (msgim.id == 0x50)
        {
          decodeFrameCAN(msgim.buf);
        }
     }
}



void eng() 
{
                myTime = millis();
                if(myTime - lastpush > 1500) 
                      {
                          if (lastIdActive == 0)
                            {
                              Time = millis();
                              engageCAN = true;
                              lastIdActive = 1;
                              relayTime = ((millis() + 1000));
                              lastpush = Time;
                            }
                          else
                            {
                              engageCAN = false;
                              lastIdActive = 0;
                            }
                      }
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
  roll =rollx;
  pitch = pitchx;
  yaw = yawx;
  if(steerConfig.InvertWAS)
    {
      roll *= -1;
    }
}
