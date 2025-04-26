float decimal_input;
float unit_input;

int32_t vbauds;

float decode_bcd_angle(uint8_t b1, uint8_t b2) {
  int sign = (b1 & 0x80) ? -1 : 1; // Bit de signe dans le premier byte (bit 7)

  int hundreds = (b1 & 0x70) >> 4;
  int tens     = (b1 & 0x0F);
  int units    = (b2 & 0xF0) >> 4;
  int decimals = (b2 & 0x0F);

  float value = (hundreds * 1000 + tens * 100 + units) + decimals;
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
                decimal_input = msgim.buf[2] / 152.0;
                unit_input = (msgim.buf[1] * 1000.0) / 152.0;
                
                pitch = (unit_input + decimal_input);
                
                if (msgim.buf[0] >= 10)
                  {
                    roll *= -1;
                  }
    
                decimal_input = msgim.buf[5] / 152.0;
                unit_input = (msgim.buf[4] * 1000.0) / 152.0;
                
                roll = (unit_input + decimal_input);
                
                if (msgim.buf[3] >= 10)
                  {
                    pitch *= -1;
                  }
    
              yaw = decode_bcd_angle(msgim.buf[6], msgim.buf[7]); // D7-D8
              
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
      int16_t roll_raw;
      int16_t pitch_raw;
      float yaw_raw;
      
  // Extraction valeurs 16 bits (Little-endian)
  roll_raw  = (buf[3] << 8) | buf[2];
  pitch_raw = (buf[5] << 8) | buf[4];
  yaw_raw   = (buf[7] << 8) | buf[6];

  // Conversion en degrés
  roll  = roll_raw  / 32768.0 * 180.0;
  pitch = pitch_raw / 32768.0 * 180.0;
  yaw = yaw_raw   / 32768 * 180;

  roll  = roll * 10;
  pitch = pitch * 10;
  yaw = yaw * 10;
  
  if(steerConfig.InvertWAS)
    {
      roll *= -1;
    }
    
//  // Échange Roll et Pitch selon la variable swapRollPitch
  if (steerConfig.IsUseY_Axis) {
    float temp = roll;
    roll = pitch;
    pitch = temp;
  }

  if (yaw < 0) yaw = 360.0 + yaw;
  //Affichage clair des résultats
//  Serial.print("Roll: ");
//  Serial.print(roll, 3);
//  Serial.print("°, Pitch: ");
//  Serial.print(pitch, 3);
//  Serial.print("°, Yaw: ");
//  Serial.print(yaw, 3);
//  Serial.println("°");
}
