#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
// 2022/08/30追記　-nis
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double vx =  0.0;
double vy =  0.0;
double vth = 0.0;
double odom_kv = 1.0;
double odom_kth = 1.0;

bool receive_flag = false;

void roverOdomCallback(const geometry_msgs::Twist::ConstPtr& rover_odom){
  vx = odom_kv * rover_odom->linear.x;
  vy = odom_kv * rover_odom->linear.y;
  vth = odom_kth * rover_odom->angular.z;
  receive_flag = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");  // ... ROS Master(roscore)にノード名「odometry_publisher」として登録

  ros::NodeHandle n;  // ... Publisher/Subscriber を作るための「ros::NodeHandle」のインスタンスを用意する（Pythonではrospy)

  ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);  // ... Publiserを作成 テンプレート引数「nav_msgs::Odometry」を与える

  ros::Subscriber odom_sub = n.subscribe("/rover_odo", 1, roverOdomCallback);  // ... Subscriverを作成 「/rover_odo」という名前でキューのサイズが「100」、メッセージを受信した時の処理が「roverOdomCallBack」という関数になるように登録

  tf::TransformBroadcaster odom_broadcaster;
  // tf2_ros::TransformBroadcaster odom_broadcaster;

  //get params
  ros::param::param<double>("odom_kv", odom_kv, 1.0);
  ros::param::param<double>("odom_kth", odom_kth, 1.0);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20);
  while(n.ok()){
    if (receive_flag)
    {
      current_time = ros::Time::now();

      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();
      // Euler
      // double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
      // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
      // Runge–Kutta
      double delta_x = vx * dt;
      double delta_y = vy * dt;
      x += delta_x * std::cos(th + vth * dt / 2) - delta_y * std::sin(th + vth * dt / 2);
      y += delta_x * std::sin(th + vth * dt / 2) + delta_y * std::cos(th + vth * dt / 2);
      th += vth * dt;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
  
      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      //publish the message
      odom_pub.publish(odom);

      last_time = current_time;
    }
    r.sleep();
    ros::spinOnce();
  }
}
