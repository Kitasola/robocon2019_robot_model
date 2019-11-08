#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

geometry_msgs::Quaternion convertEulerQuat(double roll, double pitch,
                                           double yaw) {
  tf::Quaternion quat = tf::createQuaternionFromRPY(
      roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);
  geometry_msgs::Quaternion quat_geometry;
  quaternionTFToMsg(quat, quat_geometry);
  return quat_geometry;
}

std_msgs::ColorRGBA RED, BLUE, METAL, WHITE, GREEN, BROWN, GRAY, SKY, PINK;
void initColor();

class Pole {
public:
  Pole(double x, double y, double scale_z, std_msgs::ColorRGBA color,
       const char *ns);
  void addMap(visualization_msgs::MarkerArray *data) {
    data->markers.push_back(base_left_);
    data->markers.push_back(base_right_);
    data->markers.push_back(pillar_left_);
    data->markers.push_back(pillar_right_);
    data->markers.push_back(pole_);
  }

private:
  visualization_msgs::Marker base_left_, pillar_left_, base_right_,
      pillar_right_, pole_;
};

class Line {
public:
  Line(double width, std_msgs::ColorRGBA color, const char *ns, int id);
  void addMap(visualization_msgs::MarkerArray *data, double start_x,
              double start_y, double goal_x, double goal_y) {
    visualization_msgs::Marker line = basic_;
    geometry_msgs::Point dummy;
    dummy.x = start_x;
    dummy.y = start_y;
    dummy.z = 0;
    line.points.push_back(dummy);
    dummy.x = goal_x;
    dummy.y = goal_y;
    dummy.z = 0;
    line.points.push_back(dummy);
    data->markers.push_back(line);
    ++basic_.id;
  }

private:
  visualization_msgs::Marker basic_;
};

class Box {
public:
  Box(double scale_x, double scale_y, double scale_z, std_msgs::ColorRGBA color,
      const char *ns);
  void addMap(visualization_msgs::MarkerArray *data, double x, double y,
              double z, double yaw) {
    ++basic_.id;
    basic_.pose.position.x = x;
    basic_.pose.position.y = y;
    basic_.pose.position.z = z;
    basic_.pose.orientation = convertEulerQuat(0, 0, yaw);
    data->markers.push_back(basic_);
  }

private:
  visualization_msgs::Marker basic_;
};

class Fence {
public:
  Fence(const char *ns = "fence", std_msgs::ColorRGBA color = BROWN);
  void addMap(visualization_msgs::MarkerArray *data, double len, double x,
              double y, double yaw) {
    wall_.scale.x = len;
    wall_.pose.position.x = x + wall_.scale.y / 2 * sin(yaw / 180 * M_PI);
    wall_.pose.position.y = y - wall_.scale.y / 2 * cos(yaw / 180 * M_PI);
    wall_.pose.orientation = convertEulerQuat(0, 0, yaw);
    data->markers.push_back(wall_);
    wall_.id += 2;

    base_.scale.x = len;
    base_.pose.position.x = x + base_.scale.y / 2 * sin(yaw / 180 * M_PI);
    base_.pose.position.y = y - base_.scale.y / 2 * cos(yaw / 180 * M_PI);
    base_.pose.orientation = convertEulerQuat(0, 0, yaw);
    data->markers.push_back(base_);
    base_.id += 2;
  }

private:
  visualization_msgs::Marker wall_, base_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "field");
  ros::NodeHandle n;

  // publisher
  ros::Publisher field_pub =
      n.advertise<visualization_msgs::MarkerArray>("field", 1);
  visualization_msgs::MarkerArray field;

  initColor();

  Pole pole_1000(2.85, 4.5, 1.0, BLUE, "pole_1.0");
  Pole pole_1500(2.85, 6.5, 1.5, BLUE, "pole_1.5");
  Pole pole_2000(2.85, 8.5, 2.0, BLUE, "pole_2.0");
  pole_1000.addMap(&field);
  pole_1500.addMap(&field);
  pole_2000.addMap(&field);

  Box table_shirt(0.5, 1.0, 0.3, BLUE, "table_shirt"),
      table_sheet(0.4, 0.5, 0.3, BLUE, "table_sheet"),
      table_towel(0.8, 1.5, 0.3, GRAY, "table_towel");
  table_shirt.addMap(&field, 0.25, 0.5, 0.15, 0);
  table_sheet.addMap(&field, 0.20, 1.25, 0.15, 0);
  table_towel.addMap(&field, 0, 2.25, 0.15, 0);

  Box start(1.2, 1.2, 0.002, SKY, "start"), help(1.2, 1.2, 0.05, SKY, "help"),
      field_base(5.6, 10, 0.1, GREEN, "base");
  start.addMap(&field, 1.1, 0.6, 0.001, 0);
  start.addMap(&field, 5.4, 1.8, 0.001, 0);
  start.addMap(&field, 4.2, 0.6, 0.001, 0);
  help.addMap(&field, 5.4, 0.6, 0.025, 0);
  field_base.addMap(&field, 3.2, 5.0, -0.05, 0);

  Fence fence("fence", BLUE);
  fence.addMap(&field, 6.15, 3.075, 0, 0);
  fence.addMap(&field, 10, 6.0, 5.0, 90);
  fence.addMap(&field, 5.9, 3.2, 10, 180);
  fence.addMap(&field, 7, 0.4, 6.5, 270);

  Line line_auto(0.050, WHITE, "line_auto", 0);
  line_auto.addMap(&field, 1.0, 1.2, 1.0, 10.0);
  line_auto.addMap(&field, 2.1, 3.5, 2.1, 10.0);
  line_auto.addMap(&field, 2.85, 3.5, 2.85, 10.0);
  line_auto.addMap(&field, 3.6, 3.5, 3.6, 10.0);
  line_auto.addMap(&field, 5.4, 2.4, 5.4, 10.0);
  line_auto.addMap(&field, 0.4, 1.8, 4.8, 1.8);
  line_auto.addMap(&field, 0.4, 3.5, 6.0, 3.5);
  line_auto.addMap(&field, 0.4, 5.5, 6.0, 5.5);
  line_auto.addMap(&field, 0.4, 7.5, 6.0, 7.5);
  line_auto.addMap(&field, 0.4, 9.25, 6.0, 9.25);

  ros::Rate loop_rate(1);

  while (ros::ok()) {
    field_pub.publish(field);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

Pole::Pole(double x, double y, double scale_z, std_msgs::ColorRGBA color,
           const char *ns) {
  visualization_msgs::Marker basic;
  basic.header.frame_id = "/world";
  basic.header.stamp = ros::Time::now();
  basic.ns = ns;
  basic.action = visualization_msgs::Marker::ADD;
  basic.lifetime = ros::Duration();
  basic.pose.orientation.x = 0;
  basic.pose.orientation.y = 0;
  basic.pose.orientation.z = 0;
  basic.pose.orientation.w = 1;

  base_left_ = basic;
  base_left_.id = 0;
  base_left_.type = visualization_msgs::Marker::CUBE;
  base_left_.scale.x = 0.35;
  base_left_.scale.y = 0.4;
  base_left_.scale.z = 0.3;
  base_left_.pose.position.x = x - 1.35 - 0.35 / 2.0;
  base_left_.pose.position.y = y;
  base_left_.pose.position.z = 0.15;
  base_left_.color = color;
  base_right_ = base_left_;
  base_right_.id = 1;
  base_right_.pose.position.x = x + 1.35 + 0.35 / 2.0;

  pillar_left_ = basic;
  pillar_left_.id = 2;
  pillar_left_.type = visualization_msgs::Marker::CYLINDER;
  pillar_left_.scale.x = 0.05;
  pillar_left_.scale.y = 0.05;
  pillar_left_.scale.z = scale_z + 0.1 - 0.3;
  pillar_left_.pose.position.x = x - 1.35 - pillar_left_.scale.x / 2.0;
  pillar_left_.pose.position.y = y;
  pillar_left_.pose.position.z = 0.3 + pillar_left_.scale.z / 2.0;
  pillar_left_.color = METAL;
  pillar_right_ = pillar_left_;
  pillar_right_.id = 3;
  pillar_right_.pose.position.x = x + 1.35 + pillar_left_.scale.x;

  pole_ = basic;
  pole_.id = 4;
  pole_.type = visualization_msgs::Marker::CYLINDER;
  pole_.scale.x = 0.025;
  pole_.scale.y = 0.025;
  pole_.scale.z = 3.0;
  pole_.pose.position.x = x;
  pole_.pose.position.y = y;
  pole_.pose.position.z = scale_z;
  pole_.pose.orientation = convertEulerQuat(0, -90, 0);
  pole_.color = METAL;
}

Line::Line(double width, std_msgs::ColorRGBA color, const char *ns, int id) {
  basic_.header.frame_id = "/world";
  basic_.id = id;
  basic_.ns = ns;
  basic_.header.stamp = ros::Time::now();
  basic_.type = visualization_msgs::Marker::LINE_STRIP;
  basic_.action = visualization_msgs::Marker::ADD;
  basic_.lifetime = ros::Duration();
  basic_.scale.x = width;
  basic_.pose.orientation.x = 0;
  basic_.pose.orientation.y = 0;
  basic_.pose.orientation.z = 0;
  basic_.pose.orientation.w = 1;
  basic_.color = color;
}

Box::Box(double scale_x, double scale_y, double scale_z,
         std_msgs::ColorRGBA color, const char *ns) {
  basic_.header.frame_id = "/world";
  basic_.header.stamp = ros::Time::now();
  basic_.id = 0;
  basic_.action = visualization_msgs::Marker::ADD;
  basic_.lifetime = ros::Duration();
  basic_.ns = ns;
  basic_.type = visualization_msgs::Marker::CUBE;
  basic_.color = color;
  basic_.scale.x = scale_x;
  basic_.scale.y = scale_y;
  basic_.scale.z = scale_z;
}

Fence::Fence(const char *ns, std_msgs::ColorRGBA color) {
  wall_.header.frame_id = "/world";
  wall_.header.stamp = ros::Time::now();
  wall_.id = 0;
  wall_.action = visualization_msgs::Marker::ADD;
  wall_.lifetime = ros::Duration();
  wall_.ns = ns;
  wall_.type = visualization_msgs::Marker::CUBE;
  wall_.color = color;
  base_ = wall_;
  wall_.scale.y = 0.024;
  wall_.scale.z = 0.126;
  wall_.pose.position.z = 0.087;

  base_.id = 1;
  base_.scale.y = 0.15;
  base_.scale.z = 0.024;
  base_.pose.position.z = 0.012;
}

void initColor() {
  RED.r = 1.0f;
  RED.a = 1.0f;

  BLUE.b = 1.0f;
  BLUE.a = 1.0f;

  WHITE.r = 1.0f;
  WHITE.g = 1.0f;
  WHITE.b = 1.0f;
  WHITE.a = 1.0f;

  GREEN.r = 13.0 / 255;
  GREEN.g = 97.0 / 255;
  GREEN.b = 81.0 / 255;
  GREEN.a = 1.0f;

  METAL.r = 128.0 / 255;
  METAL.g = 140.0 / 255;
  METAL.b = 136.0 / 255;
  METAL.a = 1.0f;

  GRAY.r = 128.0 / 255;
  GRAY.g = 128.0 / 255;
  GRAY.b = 128.0 / 255;
  GRAY.a = 1.0f;

  SKY.r = 111.0 / 255;
  SKY.g = 153.0 / 255;
  SKY.b = 177.0 / 255;
  SKY.a = 1.0f;

  PINK.r = 234.0 / 255;
  PINK.g = 170.0 / 255;
  PINK.b = 160.0 / 255;
  PINK.a = 1.0f;
}
