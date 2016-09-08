ifndef TF_KEYBOARD_H_
#define TF_KEYBOARD_H_

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include "sensor_msgs/Joy.h"
#include <tf/transform_broadcaster.h>


#define DEAD_ZONE 0.05
#define DEAD_ZONE_YAW 0.05


class joypad {
public:
    joypad();
	void run();
    void ps_callback(sensor_msgs::Joy msg_joy);
    void send_data();

private:

    ros::NodeHandle _nh;
    ros::Subscriber _joy_subscriber;
    ros::Publisher _joy_publisher, _joy_buttons_sub;
    ros::ServiceClient _chang_state_client;
    bool _just_buttons;
    double _x, _y, _z_pos, _z_vel, _yaw;
    std::string _req_state;
    sensor_msgs::Joy buttons_state;

};


#endif /* TS_KEYBOARD_H_ */
