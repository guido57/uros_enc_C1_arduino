// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <Arduino.h>
#include <WiFi.h>

#if !defined(ESP32) 
#error This program is only available for ESP32 Dev module or ESP32 D1 mini 
#endif

#include "ros2.h"
#include "RplidarC1.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "battery.h"
#include "wifimonitor.h"

#define LED_PIN 2

// Lidar object
extern RplidarC1 lidar;      // declared in RplidarC1.cpp
extern void motors_init();   // declared in motors.cpp

// the battery monitor
Battery *battery;

extern WiFiMonitor *wifimonitor;

void setup() {
  Serial.begin(115200);
  printf("setup ...\r\n");
  
  lidar.begin();
  delay(1000);
  lidar.resetLidar();
  delay(800);
  lidar.startLidar();

  // Initialize Battery object
  battery = new Battery(Wire);

  motors_init();
  delay(2000);

  rcl_ret_t ret;
  if(RCL_RET_OK != (ret = InitROS())){
    printf("rclc error=%d %s\r\n",ret, rcutils_get_error_string().str);
    delay(500);
    esp_restart;
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  printf("end of setup!\r\n");
}

void loop() {
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  if(ret != RCL_RET_OK){
    printf("rclc_executor_spin_some error=%d %s\r\n",ret, rcutils_get_error_string().str);
    rcl_reset_error();
  }


  
}