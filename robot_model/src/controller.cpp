#include "dead_reckoning/point.h"
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

dead_reckoning::point field_point;

void pointCheck(const dead_reckoning::point point) { field_point = point; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  ros::Subscriber field_sub = n.subscribe("field_point", 1, pointCheck);

  ros::Rate loop_rate(10);
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  while (ros::ok()) {
    ros::spinOnce();

    transform.setOrigin(tf::Vector3(field_point.x, field_point.y, 0));
    q.setRPY(0, 0, field_point.yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world",
                                          "body_link"));

    loop_rate.sleep();
  }
  return 0;
}
