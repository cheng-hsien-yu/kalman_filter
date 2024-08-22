// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>

// include math 
#include <math.h>

using namespace std;

float theta_error;

// turtle pose
turtlesim::Pose turtle;

// goal points
geometry_msgs::Point turtle_goal;

// turtle twist
geometry_msgs::Twist turtle_twist;

// turtle publisher
ros::Publisher turtle_pub;


bool reset;

struct XY{
    float x;
	float y;
};

struct XY pos_err_I;



// declare call back function
void turtle_cb(const turtlesim::Pose::ConstPtr& msg)
{
    turtle = *msg;
}



// rotate the world frame coordinate to body frame 
void worldtobody2D(float &x, float &y, float theta)
{
	// Define the rotation matrix
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    // Apply the rotation to the coordinates
    float new_x = cos_theta * x + sin_theta * y;
    float new_y = -sin_theta * x + cos_theta * y;

    // Update the input coordinates with the rotated values
    x = new_x;
    y = new_y;

	std::cout << "x rot: " << x << std::endl;
} 

void worldtobodyQuat(float &x, float &y, float theta)
{
	float w = cos(theta / 2.0);
	float z = sin(theta / 2.0);

	Eigen::Quaterniond q(w, 0, 0, z);
	Eigen::Quaterniond q_normalized(q.w()/q.norm(), q.x()/q.norm(), q.y()/q.norm(), q.z()/q.norm());
	Eigen::Quaterniond v(0, x, y, 0);
	Eigen::Quaterniond v_new = q_normalized.inverse() * v * q_normalized;
	x = v_new.x();
	y = v_new.y();

	std::cout << "x quat: " << pos_err_I.x << std::endl;
	std::cout << "y quat: " << pos_err_I.y << std::endl;
}

// P control for goal position in world frame 
void Positioncontrol(geometry_msgs::Point &goal, turtlesim::Pose &turtle_pose, geometry_msgs::Twist &turtle_vel_msg, float &error_distance) {
	
	// P control gain
	float Kp_linear = 1;
	float Kp_angular = 2;

	// error in inertia frame
	pos_err_I.x = goal.x - turtle_pose.x;
	pos_err_I.y = goal.y - turtle_pose.y;


	// Find the goal_point position in Body(turtlesim) frame
	worldtobodyQuat(pos_err_I.x, pos_err_I.y, turtle_pose.theta);
	//worldtobody2D(pos_err_I.x, pos_err_I.y, turtle_pose.theta);

	// Find the error postion 
	error_distance = sqrt(pow(pos_err_I.x, 2) + pow(pos_err_I.y, 2));

	// Find the error theta 
	float error_theta = atan2(pos_err_I.y,pos_err_I.x);

	// Output boundary
	if (error_distance > 2) error_distance = 2;

	// Design your controller here, you may use a simple P controller
	turtle_vel_msg.linear.x = Kp_linear * error_distance;
	turtle_vel_msg.angular.z = Kp_angular * error_theta;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_Pcontrol");
  	ros::NodeHandle n;

  	// declare publisher & subscriber
  	ros::Subscriber turtle_sub = n.subscribe<turtlesim::Pose>("/turtlesim/turtle1/pose", 1, turtle_cb); 
	turtle_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/turtle1/cmd_vel",1);
	
	// Define turtle goal points
	std::vector<geometry_msgs::Point> waypoints;

	geometry_msgs::Point point;
	point.x = 3.0;
	point.y = 3.0;
	waypoints.push_back(point);

	point.x = 3.0;
	point.y = 7.0;
	waypoints.push_back(point);

	point.x = 7.0;
	point.y = 7.0;
	waypoints.push_back(point);

	point.x = 7.0;
	point.y = 3.0;
	waypoints.push_back(point);
	
	int waypoint_index = 0; // Start with the first waypoint
	turtle_goal = waypoints[waypoint_index];

	// setting frequency as 10 Hz
  	ros::Rate loop_rate(10);
	
  	while (ros::ok()){
		ros::spinOnce();

		ROS_INFO("goal x : %f \t y : %f\n",turtle_goal.x,turtle_goal.y);
    	ROS_INFO("pose x : %f \t y : %f\n",turtle.x,turtle.y);
    	ROS_INFO("pose theta: %f \n",turtle.theta);

		float error_distance;
		// Input your goal_point to your controller
		Positioncontrol(turtle_goal, turtle, turtle_twist, error_distance);

		if (error_distance < 0.01) {
        	// Move to the next waypoint
        	waypoint_index = (waypoint_index + 1) % waypoints.size();
        	turtle_goal = waypoints[waypoint_index];
    	}

		turtle_pub.publish(turtle_twist);

		loop_rate.sleep();
	}
	return 0;
}