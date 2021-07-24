//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Publish function for IMU odometry, directly to /IMU
void publishIMU(double time) {
  imu_msg.header.frame_id = odom;
  imu_msg.header.seq = counter;
  imu_msg.header.stamp = nh.now();
  imu_msg.orientation.x = q0;//myImu.readQuatX();
  imu_msg.orientation.y = q1;//myImu.readQuatY();
  imu_msg.orientation.z = q2;//myImu.readQuatZ();
  imu_msg.orientation.w = q3;//myImu.readQuatW();

  imu_msg.angular_velocity.x = gx;//myImu.readGyroX();
  imu_msg.angular_velocity.y = gy;//myImu.readGyroY();
  imu_msg.angular_velocity.z = gz;//myImu.readGyroZ();

  imu_msg.linear_acceleration.x = ax;//myImu.readAccelX();
  imu_msg.linear_acceleration.y = ay;//myImu.readAccelY();
  imu_msg.linear_acceleration.z = az;//myImu.readAccelZ();

  counter++;

  imu_pub.publish(&imu_msg);
  nh.spinOnce();
  nh.loginfo("Publishing IMU");
}
