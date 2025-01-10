// RplidarC1.h
#ifndef RPLIDAR_C1_H
#define RPLIDAR_C1_H

#include <Arduino.h>
#include <sensor_msgs/msg/laser_scan.h>
#include "ros2.h"

#define BUF_SIZE 450

class RplidarC1 {
public:

    float distances[BUF_SIZE];
    float qualities[BUF_SIZE];

    RplidarC1();
    void begin();
    void resetLidar();
    void startLidar();
    int uartRx();
    void processFrame(int pos_min, int pos_max,int total_points);
    float interpolate(float x1, float y1, float x2, float y2, float x) ;

    sensor_msgs__msg__LaserScan scan_msg;
    
private:
    uint8_t DataBuffer[BUF_SIZE];
    uint16_t dataIndex;
    bool pointAlign;
    float last_angle_prev_scan; 

    struct stScanDataPoint_t {
        uint8_t quality;
        uint8_t angle_low;
        uint8_t angle_high;
        uint8_t distance_low;
        uint8_t distance_high;
    };
};

extern void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
extern void lidar_loop();

#endif
