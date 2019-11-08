#include <cmath>
#include <robot_model/status.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

ros::Time phase_clock;

void checkStatus(const robot_model::status data) {
  std::string status(data.status);
  if (status == "phase_time") {
    phase_clock = ros::Time::now() + ros::Duration(data.data);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "status");
  ros::NodeHandle n;

  ros::Subscriber status_sub = n.subscribe("robot_status", 10, checkStatus);
  ros::Publisher time_pub =
      n.advertise<std_msgs::Float32>("phase_time_status", 10);
  std_msgs::Float32 time;

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    time.data = (phase_clock - ros::Time::now()).toSec();
    if (time.data < -1.0) {
      time.data = 0;
    }
    time_pub.publish(time);

    loop_rate.sleep();
  }

  return 0;
}
