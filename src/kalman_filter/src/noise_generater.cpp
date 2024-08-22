#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <random>

ros::Publisher noisy_pose_pub;
ros::Publisher noisy_cmd_vel_pub;

std::default_random_engine generator;
std::normal_distribution<double> distribution_small(0.0, 0.1);
std::normal_distribution<double> distribution_big(0.0, 0.5);

void pose_Callback(const turtlesim::Pose::ConstPtr& msg) {
    turtlesim::Pose noisy_pose = *msg;

    // 模擬有噪聲的位置信息
    noisy_pose.x += distribution_small(generator);
    noisy_pose.y += distribution_small(generator);
    noisy_pose.theta += distribution_small(generator);
    noisy_pose.linear_velocity += distribution_small(generator);
    noisy_pose.angular_velocity += distribution_small(generator);

    // 發布有噪聲的位置信息
    noisy_pose_pub.publish(noisy_pose);
}

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg) {
    geometry_msgs::Twist noisy_cmd = *msg;

    // 為線速度與角速度添加隨機噪聲
    noisy_cmd.linear.x += distribution_big(generator);
    noisy_cmd.linear.y += distribution_big(generator);
    noisy_cmd.linear.z += distribution_big(generator);
    noisy_cmd.angular.x += distribution_big(generator);
    noisy_cmd.angular.y += distribution_big(generator);
    noisy_cmd.angular.z += distribution_big(generator);
    
    // 發布帶有噪聲的 cmd_vel 指令
    noisy_cmd_vel_pub.publish(noisy_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "noise_generator");
    ros::NodeHandle nh;

    noisy_pose_pub = nh.advertise<turtlesim::Pose>("noisy_turtle_pose", 10);
    noisy_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("noisy_turtle_cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("turtlesim/turtle1/pose", 10, pose_Callback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/turtlesim/turtle1/cmd_vel", 10, cmd_vel_Callback);

    ros::spin();
    return 0;
}
