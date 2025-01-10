// RplidarC1.cpp
#include "RplidarC1.h"
#include "ros2.h"
#include <rmw_microros/rmw_microros.h>
#include <vector>
#include <numeric>

rcl_publisher_t lidar_publisher;

RplidarC1::RplidarC1()
    : dataIndex(0), pointAlign(false), last_angle_prev_scan(-1.0) {}

void RplidarC1::begin() {
    Serial2.setRxBufferSize(12000);
    Serial2.begin(460800, SERIAL_8N1, 16, 17);
    // Initialize LaserScan message
    scan_msg.header.frame_id.data = (char *)"laser_frame";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1; // Assuming 10 Hz scan rate
    scan_msg.range_min = 0.05;
    scan_msg.range_max = 10.0;
}

void RplidarC1::resetLidar() {
    uint8_t resetCommand[] = {0xA5, 0x40};
    Serial2.write(resetCommand, sizeof(resetCommand));
    Serial.println("LIDAR reset command sent");
}

void RplidarC1::startLidar() {
    uint8_t startCommand[] = {0xA5, 0x20};
    Serial2.write(startCommand, sizeof(startCommand));
    Serial.println("LIDAR start command sent");
}

// Linear interpolation function
float RplidarC1::interpolate(float x1, float y1, float x2, float y2, float x) {
    //return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
    if(abs(x-x1) < abs(x-x2))
        return y1;
    else
        return y2;
}


int RplidarC1::uartRx() {
    unsigned long timeout = millis() + 3000;
    uint8_t bytes[5];
    pointAlign = false;
    int num_points = 0;
    int ndx = 0; // 0 ... BUF_SIZE-1
    float last_angle = last_angle_prev_scan;
    float last_distance, last_quality;
    stScanDataPoint_t * point;
    float delta_angle_interp = 360.0/BUF_SIZE;
    
    while (millis() < timeout) {
        
        if (Serial2.available() < 5)
            continue;
        
        Serial2.read(bytes,1);
        // check if we are point aligned    
        if( ((bytes[0] & 0x03) != 0x01) && ((bytes[0] & 0x03) != 0x02) ){ // S must be opposite of S^
            pointAlign = false;
            Serial.printf("bytes[0]=%x not aligned\r\n",bytes[0]);
            num_points = 0;
            continue;
        }    
        Serial2.read(bytes+1,1);
        if ((bytes[1] & 0x1) != 0x1){ // LSB must be 1
            pointAlign = false;
            Serial.printf("bytes[1]=%x not aligned\r\n",bytes[1]);
            num_points = 0;
            continue;
        }
        Serial2.read(bytes+2,3);
        pointAlign = true;
        num_points ++;    
        point = (stScanDataPoint_t *) bytes;
        // calculate angle in degrees
        uint16_t angle_int = point->angle_high;
        angle_int = (angle_int << 1) | ((point->angle_low )>> 7); 
        uint16_t angle_flo = ( point->angle_low & 0x7F ) >> 1;        
        float angle =  ((float) angle_int) + ((float)angle_flo) / 64.0;
        uint16_t distance_int = ( ((uint16_t)point->distance_high) << 8) | ((uint16_t)point->distance_low);
        float distance = (((float)distance_int)/4.0 - 242.0 )/1.1; // RPLIDAR C1 Error Correction
        float quality = point->quality / 4.0;

        if(num_points < 10 ){

            // printf("%lu num_points=%d last_angle=%f angle=%f pointAlign=%d Ser2.aval=%d\r\n",
            //   millis(), num_points,last_angle,angle,pointAlign, Serial2.available());
           
        }

        if(ndx == 0){
            printf("%lu ndx=%d num_points=%d angle=%f last_angle=%f\r\n", 
                    millis(),ndx, num_points,angle, last_angle);
        }

        float angle_interp = ndx * delta_angle_interp;
        
        if(angle_interp >= last_angle && angle_interp < angle && angle > last_angle){
        
            float distance_interp;
            float quality_interp;
            
            if(distance < 0 || last_distance < 0){
                distance = 0;
                last_distance = 0;
                quality = 0;
                last_quality = 0;
                distance_interp = 0;
                quality_interp = 0;
            }else{
                distance_interp = interpolate(last_angle,last_distance,angle, distance,angle_interp);
                quality_interp = interpolate(last_angle,last_quality,angle, quality,angle_interp);
                // distance_interp = distance;
                // quality_interp = quality;
            }
            if(ndx <= 224){
                qualities[224-ndx] = quality_interp;    
                distances[224-ndx] = distance_interp / 1000.0;    // mm to m
            }else{
                qualities[449 + 225 - ndx] = quality_interp;    
                distances[449 + 225 - ndx] = distance_interp / 1000.0;    // mm to m
            }

            if(ndx == BUF_SIZE -1){
                // scan completed!
                printf("%d points returned. num_points=%d angle=%f last_angle=%f\r\n",
                    ndx, num_points, angle,last_angle);
                last_angle_prev_scan = angle - 360.0; // it should become a little negative value (e.g. -0.7)
                return num_points;        
            }

            ndx++;
        }

        last_angle = angle;
        last_distance = distance;
        last_quality = quality;
    }    
        
    pointAlign = false;
    printf("%lu timeout\r\n",millis());
    return 0;
}


void RplidarC1::processFrame(int pos_min, int pos_max, int total_points) {
    // printf("\r\n\r\nBEFORE REORDERING...\r\n");
    // for(int i =0; i< total_points;i++)
    //     printf("%d angle=%.3f ranges=%.3f intensity=%.3f\r\n",
    //              i, angles[i], ranges[i], intensities[i]);
     
    // reorder to be sure that [0] is the minimun angle and [num_points-1] the maximum angle

    // printf("\r\n\r\nAFTER REORDERING...\r\n");
    // for(int i =0; i< total_points;i++)
    //     printf("%d angle=%.3f ranges=%.3f intensity=%.3f\r\n",
    //              i, angles[i], ranges[i], intensities[i]);
        

    // Populate the LaserScan message with the range [pos_min,pos_max]
    
    // Get the current time
    int64_t mil = rmw_uros_epoch_millis();
    int64_t nanos = rmw_uros_epoch_nanos();

    scan_msg.angle_min = 0;
    scan_msg.angle_increment = (2 * PI) / BUF_SIZE;
    scan_msg.angle_max = scan_msg.angle_increment *(BUF_SIZE - 1);
    
    scan_msg.header.stamp.sec = mil / 1000;
    scan_msg.header.stamp.nanosec = nanos % 1000000000;
    scan_msg.ranges.data = distances;
    scan_msg.ranges.size = BUF_SIZE;
    scan_msg.ranges.capacity = BUF_SIZE;
    scan_msg.intensities.data = qualities;
    scan_msg.intensities.size = BUF_SIZE;
    scan_msg.intensities.capacity = BUF_SIZE;
    // Serial.printf("angle_min=%f angle_max=%f angle_increment=%f \r\n", 
    //     scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment);   
}


RplidarC1 lidar;

// declared in motors.cpp but printed by lidar_loop()
extern float linearVelocity, angularVelocity;
extern float vL, vR;
extern float currentRpmL, currentRpmR;
extern float currentmsL, currentmsR;
extern float actuating_signal_LW, actuating_signal_RW;
extern int enc_r_total;
extern int enc_l_total;
extern float x,y,theta;

extern void update_odometry();
extern void publish_odometry(); 

unsigned long total_loop_time = 0L;
float loop_period = 0.0;
int consecutive_errors = 0;
void lidar_loop(){

    unsigned long uart_elapsed = millis();
    int count;
    count = lidar.uartRx();
    uart_elapsed = millis() - uart_elapsed;
    
    update_odometry();
    
    unsigned long process_elapsed = millis();
    //lidar.processAllFrames(count);
    process_elapsed = millis()-process_elapsed;

    unsigned long publish_elapsed = millis();
    int nc = 50;
    //for(int i=0; i< count; i+=nc){
        //lidar.processFrame(nc*(i/nc), min(count-1, nc*(i/nc)+nc-1),count);
        lidar.processFrame(0,count-1,count);
        
        rcl_ret_t ret_pub = rcl_publish(&lidar_publisher, &lidar.scan_msg, NULL);
        
        if(ret_pub != RCL_RET_OK){
            printf("rcl_publish returned %d\r\n", ret_pub);
            consecutive_errors ++;
            // if(consecutive_errors > 10)
            //     esp_restart();
        }else
            consecutive_errors = 0;
    
    //}
    publish_elapsed = millis() - publish_elapsed;

    unsigned long pub_odom_elapsed = millis();
    publish_odometry(); 
    pub_odom_elapsed = millis() - pub_odom_elapsed,

    // calculate loop period  
    total_loop_time = millis()-total_loop_time;
    float total_loop_time_f = (float) total_loop_time;
    loop_period = loop_period*0.9 + total_loop_time_f*0.1;

    uint64_t mil = rmw_uros_epoch_millis();
    uint64_t nanos = rmw_uros_epoch_nanos();
    
    //Serial.printf("millis=%lld nanos=%lld lv=%.3f av=%.3f vL=%.3f vR=%.3f encL=%d encR=%d msL=%.3f msR=%.3f LW=%.0f RW=%.0f ||lidar=%dpts uartRx=%lums Proc=%lums Pub=%lums Loop=%lums Freq=%.1fHz Ser2.av=%d\r\n",
    Serial.printf("lv=%.3f av=%.3f vL=%.3f vR=%.3f encL=%d encR=%d msL=%.3f msR=%.3f LW=%.0f RW=%.0f ||lidar=%dpts uartRx=%lums Proc=%lums Pub=%lums PubOdom=%lums Loop=%lums Freq=%.1fHz Ser2.av=%d\r\n",
        //x,y,theta,
        linearVelocity, angularVelocity, vL, vR, enc_l_total, enc_r_total, currentmsL, currentmsR, actuating_signal_LW, actuating_signal_RW,
        count, uart_elapsed, process_elapsed, publish_elapsed, pub_odom_elapsed, total_loop_time, 1000.0/loop_period, Serial2.available()
     );
    total_loop_time = millis();
}


void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL )  {

    unsigned long lidar_timer_cb_elapsed = millis(); 
    lidar_loop();
    // printf("\r\n%lu lidar_timer_callback took %lu millis. Serial2aval=%d\r\n", 
    //       millis(), millis()-lidar_timer_cb_elapsed, Serial2.available());
  }
}

