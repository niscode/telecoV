#include <jsk_rviz_plugins/Pictogram.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <math.h>

float voltage =  0.0;
// std_msgs::Float32 voltage = 0.0;
int receive_flag = 0;

void roverSensorCallback(const std_msgs::Int16MultiArray::ConstPtr& rover_sensor){
//   ROS_INFO("[VOLTAGE] origin = %d", rover_sensor->data[1]);
  voltage = rover_sensor->data[1] / float(1000);
  receive_flag = 1;
//   ROS_INFO("[VOLTAGE] subscribed = %3.2f", voltage);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "information_publisher");
  ros::NodeHandle nh;
  ros::Publisher voltage_pub = nh.advertise<std_msgs::Float32>("voltage", 50);

  // 電圧値（voltage）を読みにいく
  ros::Subscriber voltage_sub = nh.subscribe("/rover_sensor", 100, roverSensorCallback);    // Subscriverを作成 「/rover_odo」という名前でキューのサイズが「100」 (メッセージにキューが達すると新しいメッセージがくる度に古いものを捨てる)、メッセージを受信するたびに「roverOdomCallBack」という関数を呼び出す


  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float32 float32;
    // float32.data = sin(0.02 * count * 2 * M_PI);
    float32.data = voltage;
    voltage_pub.publish(float32);
    // ROS_INFO("[VOLTAGE] Published = %3.2f", float32.data);

    count++;
    ros::spinOnce();  // その行が実行されるとCallback関数にアクセス
    loop_rate.sleep();
  }
  return 0;
}