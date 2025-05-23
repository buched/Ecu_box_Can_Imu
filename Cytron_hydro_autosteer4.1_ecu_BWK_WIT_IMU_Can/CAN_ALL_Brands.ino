float decimal_input;
float unit_input;

int32_t vbauds;

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

void CAN_setup (void) 
{
    vbauds = 250000;
    I_Bus.begin();
    I_Bus.setBaudRate(vbauds);
    delay(500);
    V_Bus.begin();
    V_Bus.setBaudRate(vbauds);
    delay(500);
}

//---Receive K_Bus message
void VBus_Receive()
{
CAN_message_t msgiv;
      if ( V_Bus.read(msgiv) ) 
          {
            if (msgiv.id == 0x18FF1A03)
              {
                if ((msgiv.buf[2])== 0x15)
                  {
                    Time = millis();
                    engageCAN = 1;
                    relayTime = ((millis() + 1000));
                   }
               }
          }
}
//---Receive K_Bus message
void Imu_Receive()
{
CAN_message_t msgim;
      if ( I_Bus.read(msgim) ) 
          {
            if (msgim.id == 0x585)
              {
              roll = decodeAngleXY(&msgim.buf[0]);
              pitch = decodeAngleXY(&msgim.buf[3]);                  
              yaw = decode_bcd_angle(msgim.buf[6], msgim.buf[7]); // D7-D8

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
