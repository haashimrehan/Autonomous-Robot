boolean pointToBlock (Block target, int hedge) {//Points to a Block that is sent in
  if (cam.getBlock()) { //Check if there is blocks
    return alignRobot(target, hedge);
  } else {
    delay(20);
    if (cam.getBlock()) { //check again
      return alignRobot(target, hedge);
    } else { //There are no blocks
      Serial.println("No Block Detected");
    }
  }
}

boolean alignRobot(Block target, int hedge) {
  int width = 320; // pixy cam width
  if (target.x > width / 2 + hedge) {
    //Turns Right until the blocks x val is within a range
    turn(1);
    Serial.println("RIGHT");
  }
  else if (target.x < width / 2 - hedge) {
    //Turns Left until the blocks x val is within a range
    Serial.println("LEFT");
    turn(-1);
  }
  else if (target.x > width / 2 - hedge && target.x < width / 2 + hedge) {
    //Stops once the block is within a range
    Serial.println("CENTER");
    drive2(0);
    return true;
  }
  return false;
}
