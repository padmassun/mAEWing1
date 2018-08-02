byte checksum;
byte sendbuff[3] = {00,224,01};
int byteavail;
int servonum;
byte junk;
float tempangle;
float zeroangle;
int steps = 11;
float angles[11];
//float pwmvals[11] = {1120,1220,1320,1420,1500,1520,1540,1620,1720,1820,1920}; // Left Side
float pwmvals[11] = {1920,1820,1720,1620,1540,1520,1500,1420,1320,1220,1120}; // Right Side


union{
  int32_t val;
  byte b[4];
}angle;

void setup() {
  // setting up serial monitor
  Serial.begin(9600);

  // setting up HW serial for talking with inclinometer
  Serial1.begin(115200);
}

void loop() {
  delay(1000);

  // 23,22,21, 3, 4, 5, 6,10,32
  // L1,L2,L3,R1,L4,R2,R3,R4,DTHR
  Serial.println("L4 = 4; L3 = 21; L2 = 22; L1 = 23; R1 = 3; R2 = 5; R3 = 6; R4 = 10; DTHR = 32;");
  Serial.println("Enter Surface number for calibration:");
  while (Serial.available() == 0) ;  // Wait here until input buffer has a character
  {
    servonum = Serial.parseInt();
    Serial.print("PWM output = "); Serial.println(servonum, DEC);

    while (Serial.available() > 0)
    { junk = Serial.read() ; }      // clear the keyboard buffer
  }
  
  Serial.println("Beginning Test");
  Serial.println("Hold Surface at Zero Position");
  delay(2000);
  tempangle = 0;
  for(int i = 0; i < 100; i++){

    // clear the buffer just in case
    //HWSERIAL.flush();
    while (Serial1.available() > 0)
    { junk = Serial1.read() ; }      // clear the keyboard buffer

    // write command to receive angle
    Serial1.write(sendbuff[2]);
    Serial1.write(sendbuff[1]);
    Serial1.write(sendbuff[0]);

    // wait for the buffer to fill
    delay(20);

    // see how many bytes in buffer
    byteavail = Serial1.available();

    // fill buffer
    angle.b[3] = Serial1.read();
    angle.b[2] = Serial1.read();
    angle.b[1] = Serial1.read();
    angle.b[0] = Serial1.read();
    
    tempangle +=angle.val/1000.0;
  }

  zeroangle = tempangle/100;
  Serial.print("Zero position: ");
  Serial.println(zeroangle);
  Serial.println("Switch to Auto Mode");
  Serial.println("Hands Clear!");
  delay(5000);
  Serial.println("Calibration Starting");

  for(int i = 0; i < steps; i++){
    analogWriteResolution(16);
    analogWriteFrequency(servonum, 333);
    analogWrite(servonum, pwmvals[i]/3003.0*65535.0);
    delay(2000);
    tempangle = 0;
    for(int j = 0; j < 100; j++){

      // clear the buffer just in case
      //HWSERIAL.flush();
      while (Serial1.available() > 0)
      { junk = Serial1.read() ; }      // clear the keyboard buffer

      // write command to receive angle
      Serial1.write(sendbuff[2]);
      Serial1.write(sendbuff[1]);
      Serial1.write(sendbuff[0]);

      // wait for the buffer to fill
      delay(20);

      // see how many bytes in buffer
      byteavail = Serial1.available();

      // fill buffer
      angle.b[3] = Serial1.read();
      angle.b[2] = Serial1.read();
      angle.b[1] = Serial1.read();
      angle.b[0] = Serial1.read();

      tempangle +=angle.val/1000.0;
    }

    angles[i] = tempangle/100 - zeroangle;
  }

  // Return to "Off"
  analogWrite(servonum,0.0);
    
  Serial.println("Calibration Complete");
  Serial.println();
  
Serial.print("pwm_us = [");
  for(int i = 0; i < steps; i++){
    Serial.print(pwmvals[i]);
    Serial.print("  ");
  }
Serial.println("];");

Serial.print("angle_deg = -[");
  for(int i = 0; i < steps; i++){
    Serial.print(angles[i]);
    Serial.print("  ");
  }
Serial.println("]; % Ensure Inclinometer and AC convention are the same!!");
Serial.println("fitOrder = 2; % Start with quadratic, alter as required");
Serial.println("[polyCoef, pwmZero_us] = SurfCal(pwm_us, angle_deg, fitOrder);");
Serial.println();
Serial.println();

  }

