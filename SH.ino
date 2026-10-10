void SH() {
  servo(1, 40);
  delay(100);
  servo(2, 0);
  delay(100);
  setMotors(-40, 40);
  delay(150);
  L1();

  /////////////////////////////////////////////////////////////////////////
  RunF(0.008, 0.0, 0.13, 60, 300);
  RunT(0.008, 0.0, 0.13, 30);
  R1();

  RunF(0.008, 0.0, 0.13, 80, 1500);
  RunZ(0.007, 0.0, 0.13, 60);
  setMotors(60, 60);
  delay(100);
  setMotors(0, 0);
  motorStopAll();
  RunT(0.008, 0.0, 0.13, 40);
  L1();
  RunZ(0.007, 0.0, 0.13, 60);
  setMotors(60, 60);
  delay(100);
  setMotors(0, 0);
  motorStopAll();
  RunT(0.008, 0.0, 0.13, 50);
  L1();

  RunZ(0.007, 0.0, 0.13, 45);
  RunF_IMU_Time(40, 300);
  servo(2, 89);
  delay(100);
  servo(1, 0);
  delay(100);
  servo(2, 0);
  RunF_IMU_Time(-40, 600);
  turnLeft90(40);

  RunF(0.0075, 0.0, 0.13, 40, 100);
  RunZ(0.007, 0.0, 0.13, 60);
  setMotors(60, 60);
  delay(100);
  setMotors(0, 0);
  motorStopAll();
  RunT(0.008, 0.0, 0.13, 40);
  R1();
  RunZ(0.007, 0.0, 0.13, 60);
  setMotors(60, 60);
  delay(100);
  setMotors(0, 0);
  motorStopAll();
  RunF(0.0075, 0.0, 0.13, 40, 100);
  RunF(0.008, 0.0, 0.13, 80, 1480);
  RunL(0.008, 0.0, 0.13, 40);
  L1();
  RunF(0.014, 0.0, 0.13, 60, 300);
  RunT(0.007, 0.0, 0.07, 30);
  servo(1, 0);
  servo(2, 89);
  setMotors(0, 0);
  motorStopAll();
  delay(100);

}
