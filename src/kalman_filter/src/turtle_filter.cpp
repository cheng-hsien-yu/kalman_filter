#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include "kalman_filter/kalman_filter.h"

ros::Publisher kalman_pose_pub;
ros::Subscriber noisy_pose_sub;
ros::Subscriber noisy_cmd_sub;
ros::Subscriber true_pose_sub;

KalmanFilter kf(3, 2, 2);  // x = [x, y, theta] | z = [x, y] | u = [linear_vel, angular_vel]
Eigen::VectorXd x(3);       // State vector
Eigen::MatrixXd A(3, 3);    // State transition matrix
Eigen::MatrixXd B(3, 2);    // Control matrix
Eigen::MatrixXd H(2, 3);    // Measurement matrix
Eigen::MatrixXd Q(3, 3);    // Process noise covariance
Eigen::MatrixXd R(2, 2);    // Measurement noise covariance
Eigen::MatrixXd P(3, 3);    // Estimate covariance

bool use_true_measurement = false;

void initializeKalmanFilter() {
    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // 初始 B 矩陣，將控制指令轉換到全局坐標系
    B << 0, 0,
         0, 0,
         0, 0;

    H << 1, 0, 0,
         0, 1, 0;

    Q << 0.01, 0, 0,
         0, 0.01, 0,
         0, 0, 0.01;

    R << 0.25, 0,
         0, 0.25;

    P << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    x << 0, 0, 0;  // 初始狀態設置為0
    kf.init(x, P, R, Q);
}

void noisyPoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    if (!use_true_measurement) {
        Eigen::VectorXd z_meas(2);
        z_meas << msg->x, msg->y;
        kf.update(H, z_meas);
    }
}

void noisyCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double dt = 0.1;  // 設置時間步長（根據實際情況調整）
    Eigen::VectorXd u(2);
    u << msg->linear.x, msg->angular.z;

    // 更新 B 矩陣，將控制指令轉換到全局坐標系
    Eigen::MatrixXd B(3, 2);
    B << dt * cos(x(2)), 0,
         dt * sin(x(2)), 0,
         0, dt;

    kf.predict(A, B, u);

    // 將濾波後的結果發布出來
    turtlesim::Pose kalman_pose;
    kalman_pose.x = kf.getState()(0);
    kalman_pose.y = kf.getState()(1);
    kalman_pose.theta = kf.getState()(2);
    kalman_pose_pub.publish(kalman_pose);
}

void truePoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    if (use_true_measurement) {
        // 重新初始化 Kalman 濾波器
        Eigen::VectorXd z_meas(2);
        z_meas << msg->x, msg->y;

        // 使用真實狀態值重新初始化狀態向量
        x << msg->x, msg->y, msg->theta;

        // 估計協方差矩陣 P 也可以被重置（假設真實狀態測量值有很高的信任度）
        P.setIdentity();  // 可以根據需要設置為其他值

        kf.init(x, P, R, Q);
        ROS_INFO("Kalman filter reinitialized with true state.");

        // 在更新之前重新進行 Kalman 濾波器的狀態預測
        kf.update(H, z_meas);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman_filter_node");
    ros::NodeHandle nh;

    kalman_pose_pub = nh.advertise<turtlesim::Pose>("kalman_turtle_pose", 10);
    noisy_pose_sub = nh.subscribe("noisy_turtle_pose", 10, noisyPoseCallback);
    noisy_cmd_sub = nh.subscribe("/noisy_turtle_cmd_vel", 10, noisyCmdVelCallback);
    true_pose_sub = nh.subscribe("/turtlesim/turtle1/pose", 10, truePoseCallback);

    initializeKalmanFilter();

    ros::Timer timer = nh.createTimer(ros::Duration(10.0), [](const ros::TimerEvent&) {
        use_true_measurement = true;  // 開啟真實位置測量
        ros::Duration(0.1).sleep();   // 暫停0.1秒，模擬瞬間量測
        use_true_measurement = false; // 量測完成後關閉真實位置測量
    });

    ros::spin();
    return 0;
}
