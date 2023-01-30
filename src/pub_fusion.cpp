#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class OdometryPublisher
{

public:
    OdometryPublisher(ros::NodeHandle &nh) : 
        nh_(nh)
    {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("combined_odom", 5, this);
        odom_sub_ = nh_.subscribe("pub_odom", 1, &OdometryPublisher::odom_cb, this);
        imu_sub_ = nh_.subscribe("pub_imu", 1, &OdometryPublisher::imu_cb, this);
    }

    void odom_cb(const nav_msgs::OdometryConstPtr &msg){
        received_odom_ = msg;
    }

    void imu_cb(const sensor_msgs::ImuConstPtr &msg){
        received_imu_ = msg;
    }

    void run(){
        ros::Rate r(100.0);
        tf::TransformBroadcaster odom_broadcaster;

        while(nh_.ok()){
            if(received_odom_ && received_imu_){
                ros::Time time = ros::Time::now();
                ROS_INFO_STREAM("time = " << time);
                ROS_INFO_STREAM("received_odom.stamp = " << received_odom_->header.stamp);
                ROS_INFO_STREAM("received_imu.stamp = " << received_imu_->header.stamp);
                nav_msgs::Odometry odom = *received_odom_;
                geometry_msgs::TransformStamped odom_trans;

                odom.header.stamp    = odom_trans.header.stamp = time;
                odom.header.frame_id = odom_trans.header.frame_id = "odom";
                odom.child_frame_id  = odom_trans.child_frame_id  = "base_link";

                odom.pose.pose.orientation = odom_trans.transform.rotation = received_imu_->orientation;
                odom_trans.transform.translation.x = odom.pose.pose.position.x;
                odom_trans.transform.translation.y = odom.pose.pose.position.y;
                odom_trans.transform.translation.z = odom.pose.pose.position.z;

                odom_broadcaster.sendTransform(odom_trans);
                odom_pub_.publish(odom);
            }

            ros::spinOnce();
            r.sleep();
        }
    }

private:
    ros::Publisher odom_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    nav_msgs::OdometryConstPtr received_odom_;
    sensor_msgs::ImuConstPtr received_imu_;
    //ros::Duration max_tolerance_;
    ros::NodeHandle &nh_;

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "combine_dr_measurements");

    ros::NodeHandle n;
    OdometryPublisher odom_publisher(n);
    odom_publisher.run();

}