#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"


#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <tum_ics_skin_descr/Patch/TfMarkerDataPatches.h>

#include <tum_ics_skin_msgs/SkinCellDataArray.h>
#include <exo_control/exo_pos_control.h>
#include <exo_control/exo_force_control.h>
#include <iostream>
#include <cmath>
#include <typeinfo>
#include <unistd.h>
/**
 * Converts from degrees to rad.
 *
 * @param degree Value to be converted.
 * @return Value in radians.
 */
float deg2rad(float degree){
    return (degree * 3.14159265359/180);
}
/**
 * Converts from rad to deg.
 *
 * @param radian Value to be converted.
 * @return Value in degrees.
 */
float rad2deg(float radian)
{
    float pi = 3.14159;
    return(radian * (180 / pi));
}
/**
 * Main function for exo control.
 * 
 */
int main( int argc, char** argv )
{
    // Init ros.
    ros::init(argc, argv, "servo",ros::init_options::AnonymousName);
    ros::NodeHandle n;

    ros::Publisher servo_pub = n.advertise<std_msgs::Float32MultiArray>("servo", 10);
    // ros::Publisher q_state_pub = n.advertise<std_msgs::Float64>("q_state", 10);

    ros::Rate loop_rate(200);
    float delta_t = 1/(float)200; 

    // Load params from yaml file.
    float L1;
    std::string ns="~L1";
    std::stringstream s;
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L1);
    float L2;
    ns="~L2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L2);
    float L3;
    ns="~L3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L3);
    float m2;
    ns="~m2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m2);
    float m3;
    ns="~m3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m3);
    float b1;
    ns="~b1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),b1);
    float k1;
    ns="~k1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),k1);
    float theta1;
    ns="~theta1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),theta1);
    float I233;
    ns="~I233";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I233);
    float I333;
    ns="~I333";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I333);
    float g;
    ns="~g";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),g);

    // Init angle params. 
    float q1 = deg2rad(45);
    float qd1 = 0;
    float qdd1 = 0;

    float q2 = deg2rad(0); 
    float qd2 = 0;
    float qdd2 = 0; 
    
    // Mode selection.
    std::cout << "Enter start angle in deg: ";
    float start_q1 ;
    std::cin >> start_q1;
    q1 = deg2rad(start_q1);
    q2 = deg2rad(start_q1);
    // Set target position for modes with position control.
    
    
    // Send exo home.
    std_msgs::Float32MultiArray q_array;
    std_msgs::Float64 q_result;
    q_array.data.clear();
    q_array.data.push_back(rad2deg(q1));
    q_array.data.push_back(rad2deg(q2));
    servo_pub.publish(q_array);

    ros::spinOnce();
    loop_rate.sleep();
    // Sleeps for 3 second.
    sleep(3);

    // Main Loop.
    while(ros::ok())
    {   
        
        // Default q1 update 
        q1 = delta_t + q1;

        // Default q2 update 
        q2 = delta_t + q2;
        
        // Print actual angle values.
        std::cout << rad2deg(q1)<< ", "<< rad2deg(q2)<< "\n";

        // Add q1 and q1 to array for ESP32.
        q_array.data.clear();
        q_array.data.push_back(rad2deg(q1));
        q_array.data.push_back(rad2deg(q2));

        // Stop if overshoot.
        if (rad2deg(q1) >= 180.0 ){
        std::cout << "nan found";
        break;
        }
        if (rad2deg(q2) >= 180.0 ){
        std::cout << "nan found";
        break;
        }
        if (isnan(q1)&& rad2deg(q1) <= 0.0 ){
        std::cout << "nan found";
        break;
        }
        if (isnan(q2)&& rad2deg(q2) <= 0.0 ){
        std::cout << "nan found";
        break;
        }

        // Publish q state and angle array.
        double q_state = rad2deg(q1)*0.5 + 10.0;

        std::cout << q_state<<"\n";
        q_result.data = q_state;
        servo_pub.publish(q_array);
        // q_state_pub.publish(q_result);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
