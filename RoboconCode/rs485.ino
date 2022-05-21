void trans(byte data[], byte sizedata, int sizereply) {
  //byte respon[sizereply];
  //  if (rs485.available() > 0) {
  //    String xx = rs485.readString();
  //  }
//  for (int i = 0; i < 49; i++) {
//    mreply[i] = 0;
//  }
  if (rs485.availableForWrite() >= sizedata) {
    rs485.write(data, sizedata);
  }
  rs485.flush();
  while (rs485.available() == 0) {
    delay(1);
  }
  if (rs485.available() > 0) {
    rs485.readBytes(mreply, sizereply);
  }
  for (int i = 0; i < sizereply; i++) {
  //  Serial.print(mreply[i], HEX);
  //  Serial.print(" ");
  }
 // Serial.println("");
  delay(1);
}


void run_speed(byte mId, float velo) {
  long sum = 0;
  int32_t speed = velo * 100;
  byte minc[14] = {0x3e, 0xa2, mId, 0x04};
  minc[4] = 0x3e + 0xa2 + mId + 0x04;

  for (int i = 5; i < 9; i++) {
    minc[i] = speed >> 8 * (i - 5);
    sum += minc[i];
  }
  minc[9] = sum;
  trans(minc, 10, 13);
}
