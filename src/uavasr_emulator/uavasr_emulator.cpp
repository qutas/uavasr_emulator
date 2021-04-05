#include <ros/ros.h>

#include <uavasr_emulator/uavasr_emulator.h>

#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <string>
#include <math.h>
#include <bitset>
#include <algorithm>

#include <eigen3/Eigen/Dense>

#define GRAV 9.80665

namespace uavasr_emulator {

UAVASREmulator::UAVASREmulator() :
	nh_(),
	nhp_("~"),
	param_frame_id_("map"),
	param_model_id_("uav"),
	param_rate_pose_(50.0),
	param_rate_state_(1.0),
	param_rate_battery_(1.0),
	param_w0_xy_(1.0),
	param_w0_z_(1.0),
	param_w0_psi_(1.0),
	param_max_vel_xy_(2.5),
	param_max_vel_z_(1.0),
	param_auto_disarm_(0.2),
	height_max_(0.0),
	dyncfg_settings_(ros::NodeHandle(nhp_)) {

	dyncfg_settings_.setCallback(boost::bind(&UAVASREmulator::callback_cfg_settings, this, _1, _2));

	//Parameters
	bool start_armed = false;
	bool start_mode_offboard = false;
	nhp_.param("start_armed", start_armed, start_armed);
	nhp_.param("start_mode_offboard", start_mode_offboard, start_mode_offboard);

	system_armed_ = start_armed;
	system_mode_ = start_mode_offboard ? "OFFBOARD" : "CMODE(0)";

	//TODO: Starting location and yaw

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);

	nhp_.param("auto_disarm_height", param_auto_disarm_, param_auto_disarm_);

	nhp_.param("rate_state", param_rate_state_, param_rate_state_);
	nhp_.param("rate_battery", param_rate_battery_, param_rate_battery_);
	nhp_.param("rate_pose", param_rate_pose_, param_rate_pose_);

	nhp_.param("w0_xy", param_w0_xy_, param_w0_xy_);
	nhp_.param("w0_z", param_w0_z_, param_w0_z_);
	nhp_.param("w0_psi", param_w0_psi_, param_w0_psi_);
	nhp_.param("max_vel_xy", param_max_vel_xy_, param_max_vel_xy_);
	nhp_.param("max_vel_z", param_max_vel_z_, param_max_vel_z_);

	//Publishers
	pub_pose_ = nhp_.advertise<geometry_msgs::PoseStamped>( "pose", 10 );
	pub_odom_ = nhp_.advertise<nav_msgs::Odometry>( "odom", 10 );
	pub_state_ = nhp_.advertise<mavros_msgs::State>( "state", 10 );
	pub_battery_ = nhp_.advertise<sensor_msgs::BatteryState>( "battery", 10 );

	pub_target_ = nhp_.advertise<mavros_msgs::PositionTarget>( "setpoint_raw/target_local", 10 );

	//Subscribers
	sub_target_ = nhp_.subscribe<mavros_msgs::PositionTarget>( "setpoint_raw/local", 10, &UAVASREmulator::callback_position_target, this );

	//Services
	srv_arming_ = nhp_.advertiseService("cmd/arming", &UAVASREmulator::callback_arming, this);
	srv_set_mode_ = nhp_.advertiseService("set_mode", &UAVASREmulator::callback_set_mode, this);

	//Variables
	p_ = Eigen::Vector3d::Zero();
	v_ = Eigen::Vector3d::Zero();
	yaw_ = 0.0;

	/*
	position_goal_.type_mask = TYPE_MASK_POS_MODE;
	position_goal_.position.x = p_.x();
	position_goal_.position.y = p_.y();
	position_goal_.position.z = p_.z();
	position_goal_.velocity.x = v_.x();
	position_goal_.velocity.y = v_.y();
	position_goal_.velocity.z = v_.z();
	position_goal_.acceleration_or_force.x = 0.0;
	position_goal_.acceleration_or_force.y = 0.0;
	position_goal_.acceleration_or_force.z = 0.0;
	position_goal_.yaw = yaw_;
	position_goal_.yaw_rate = 0.0;
	*/

	//Start the control loop
	timer_pose_ = nhp_.createTimer(ros::Duration(1.0/param_rate_pose_), &UAVASREmulator::callback_pose, this );
	timer_state_ = nhp_.createTimer(ros::Duration(1.0/param_rate_state_), &UAVASREmulator::callback_state, this );
	timer_battery_ = nhp_.createTimer(ros::Duration(1.0/param_rate_battery_), &UAVASREmulator::callback_battery, this );

	ROS_INFO("UAVASR emulator started!");
}

UAVASREmulator::~UAVASREmulator() {
}

void UAVASREmulator::callback_cfg_settings( uavasr_emulator::EmulatorParamsConfig &config, uint32_t level ) {
	param_rate_pose_ = config.rate_pose;
	param_rate_state_ = config.rate_state;
	param_rate_battery_ = config.rate_battery;

	param_auto_disarm_ = config.auto_disarm_height;

	param_w0_xy_ = config.w0_xy;
	param_w0_z_ = config.w0_z;
	param_w0_psi_ = config.w0_psi;
	param_max_vel_xy_ = config.max_vel_xy;
	param_max_vel_z_ = config.max_vel_z;

	timer_pose_.setPeriod(ros::Duration(1.0/param_rate_pose_));
	timer_state_.setPeriod(ros::Duration(1.0/param_rate_state_));
	timer_battery_.setPeriod(ros::Duration(1.0/param_rate_battery_));
}

//TODO: There's got to be a better way to do this
double UAVASREmulator::yaw_error_shortest_path(const double y_sp, const double y) {
	double ye = y_sp - y;

	while(fabs(ye) > M_PI)
		ye += (ye > 0.0) ? -2*M_PI : 2*M_PI;

	return ye;
}

void UAVASREmulator::callback_pose(const ros::TimerEvent& e) {
	double dt = 1.0/param_rate_pose_;

	//== Get preset setpoint (hold current)
	uint16_t type_mask = TYPE_MASK_POS_MODE;
	Eigen::Vector3d x_sp = p_;
	Eigen::Vector3d v_sp = Eigen::Vector3d::Zero();
	double yaw_sp = yaw_;
	double yawd_sp = 0.0;

	//Check to see if we have a valid mode, extract data
	if( system_armed_ && (position_goal_.header.stamp > ros::Time(0)) ) {
		type_mask = position_goal_.type_mask;

		//Mode settings
		if(type_mask == TYPE_MASK_POS_MODE) {
			//Calculate error terms
			x_sp = Eigen::Vector3d(position_goal_.position.x, position_goal_.position.y, position_goal_.position.z);
			v_sp = Eigen::Vector3d::Zero();	//XXX: Regulate to position
		} else if(type_mask == TYPE_MASK_VEL_MODE) {
			//Calculate error terms
			//XXX: Eigen::Vector3d x_sp = p_;	//No position error, set above
			v_sp = Eigen::Vector3d(position_goal_.velocity.x, position_goal_.velocity.y, position_goal_.velocity.z);
		}
	}

	//== Position "Control"
	//XXX: Critical damping gains
	Eigen::Vector3d kp(param_w0_xy_*param_w0_xy_, param_w0_xy_*param_w0_xy_, param_w0_z_*param_w0_z_);
	Eigen::Vector3d kv(2*param_w0_xy_, 2*param_w0_xy_, 2*param_w0_z_);

	//"Control Law"
	//Eigen::Vector3d ex = x_sp - p_;
	//Eigen::Vector3d exd = v_sp - v_;
	Eigen::Vector3d xdd = kp.cwiseProduct(x_sp - p_) + kv.cwiseProduct(v_sp - v_);

	//Accelerate!
	v_ += xdd * dt;
	//(within limits)
	v_.x() = clamp(v_.x(), -param_max_vel_xy_, param_max_vel_xy_);
	v_.y() = clamp(v_.y(), -param_max_vel_xy_, param_max_vel_xy_);
	v_.z() = clamp(v_.z(), -param_max_vel_z_, param_max_vel_z_);

	//Integrate!
	p_ += v_ * dt;
	//Update our max height (for auto disarm
	if( p_.z() > height_max_)
		height_max_ = p_.z();
	//And don't fall through the ground
	if( p_.z() <= 0.0) {
		p_.z() = 0.0;
		v_.z() = 0.0;

		//If automatic disarm is enabled...
		//...and our flight height was at least higher than this value
		if( (param_auto_disarm_ > 0.0) && (height_max_ >= param_auto_disarm_) ) {
			//...And we are currently grounded
			height_max_ = 0.0;
			system_armed_ = false;
			ROS_INFO("Landing detected! automatic disarm activated.");
		}
	}


	//== Yaw "Control"
	/*
	Eigen::Quaterniond q_sp = Eigen::Quaterniond(
		Eigen::AngleAxisd(msg_out_target.yaw, Eigen::Vector3d::UnitZ())
	).normalized().slerp(turn_rate, q_c);
	*/
	//TODO: Have a rotation rate!
	yaw_ = position_goal_.yaw;
	Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()));


	//==-- Fill out message details
	mavros_msgs::PositionTarget msg_out_target;
	msg_out_target.header.frame_id = param_frame_id_;
	msg_out_target.header.stamp = ros::Time::now();
	msg_out_target.type_mask = type_mask;
	msg_out_target.position.x = x_sp.x();
	msg_out_target.position.y = x_sp.y();
	msg_out_target.position.z = x_sp.z();
	msg_out_target.velocity.x = v_sp.x();
	msg_out_target.velocity.y = v_sp.y();
	msg_out_target.velocity.z = v_sp.z();
	msg_out_target.acceleration_or_force.x = xdd.x();
	msg_out_target.acceleration_or_force.y = xdd.y();
	msg_out_target.acceleration_or_force.z = xdd.z();
	msg_out_target.yaw = yaw_sp;
	msg_out_target.yaw_rate = yawd_sp;

	//Calculate body-frame velocities for odom
	Eigen::Matrix3d R(q);
	Eigen::Vector3d bv = R.inverse()*v_;

	//Odometery
	nav_msgs::Odometry msg_out_odom;
	msg_out_odom.header.stamp = e.current_real;
	msg_out_odom.header.frame_id = param_frame_id_;
	msg_out_odom.child_frame_id = param_model_id_;

	msg_out_odom.pose.pose.position.x = p_.x();
	msg_out_odom.pose.pose.position.y = p_.y();
	msg_out_odom.pose.pose.position.z = p_.z();
	msg_out_odom.pose.pose.orientation.x = q.x();
	msg_out_odom.pose.pose.orientation.y = q.y();
	msg_out_odom.pose.pose.orientation.z = q.z();
	msg_out_odom.pose.pose.orientation.w = q.w();
	msg_out_odom.twist.twist.linear.x = bv.x();
	msg_out_odom.twist.twist.linear.y = bv.y();
	msg_out_odom.twist.twist.linear.z = bv.z();
	msg_out_odom.twist.twist.angular.x = 0.0;	//Not worth simulating this with the current setup
	msg_out_odom.twist.twist.angular.y = 0.0;
	msg_out_odom.twist.twist.angular.z = 0.0;

	//Pose
	geometry_msgs::PoseStamped msg_out_pose;
	msg_out_pose.header = msg_out_odom.header;
	msg_out_pose.pose = msg_out_odom.pose.pose;

	//Transform
	geometry_msgs::TransformStamped msg_out_tf;
	msg_out_tf.header = msg_out_odom.header;
	msg_out_tf.child_frame_id = param_model_id_;
	msg_out_tf.transform.translation.x = msg_out_odom.pose.pose.position.x;
	msg_out_tf.transform.translation.y = msg_out_odom.pose.pose.position.y;
	msg_out_tf.transform.translation.z = msg_out_odom.pose.pose.position.z;
	msg_out_tf.transform.rotation = msg_out_odom.pose.pose.orientation;

	//Publish
	pub_target_.publish(msg_out_target);
	pub_pose_.publish(msg_out_pose);
	pub_odom_.publish(msg_out_odom);
	tfbr_.sendTransform(msg_out_tf);
}

void UAVASREmulator::callback_state(const ros::TimerEvent& e) {
	mavros_msgs::State msg_out_state;

	msg_out_state.header.stamp = e.current_real;
	msg_out_state.header.frame_id = param_frame_id_;
	msg_out_state.connected = true;
	msg_out_state.armed = system_armed_;
	msg_out_state.guided = true;
	msg_out_state.mode = system_mode_;
	msg_out_state.system_status = 4;	//XXX: MAV_STATE_ACTIVE

	pub_state_.publish(msg_out_state);
}

void UAVASREmulator::callback_battery(const ros::TimerEvent& e) {
	sensor_msgs::BatteryState msg_out_battery;

	//random voltages around the 4.0V mark
	double v1 = 0.02*((rand() % 10) - 5) + 4;
	double v2 = 0.02*((rand() % 10) - 5) + 4;
	double v3 = 0.02*((rand() % 10) - 5) + 4;
	msg_out_battery.header.stamp = e.current_real;
	msg_out_battery.header.frame_id = param_frame_id_;
	msg_out_battery.voltage = v1 + v2 + v3;
	msg_out_battery.current = NAN;
	msg_out_battery.capacity = NAN;
	msg_out_battery.design_capacity = 4.2;
	msg_out_battery.percentage = (msg_out_battery.voltage - 11.1) / (1.5);
	msg_out_battery.power_supply_status = msg_out_battery.POWER_SUPPLY_STATUS_DISCHARGING;
	msg_out_battery.power_supply_health = msg_out_battery.POWER_SUPPLY_HEALTH_GOOD;
	msg_out_battery.power_supply_technology = msg_out_battery.POWER_SUPPLY_TECHNOLOGY_LIPO;
	msg_out_battery.cell_voltage.push_back(v1);
	msg_out_battery.cell_voltage.push_back(v2);
	msg_out_battery.cell_voltage.push_back(v3);
	msg_out_battery.location = "primary";
	msg_out_battery.location = "DEADBEEF";

	pub_battery_.publish(msg_out_battery);
}

bool UAVASREmulator::callback_arming(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res) {
	system_armed_ = req.value;

	ROS_INFO("System %s!", system_armed_ ? "armed" : "disarmed");

	res.success = true;
	res.result = 0;

	return true;
}

bool UAVASREmulator::callback_set_mode(mavros_msgs::SetMode::Request &req, mavros_msgs::SetMode::Response &res) {
	//XXX: Ignore base_mode, use PX4 custom modes
	//See if the requested mode is in our list
	if (std::find(mode_names.begin(), mode_names.end(), req.custom_mode) != mode_names.end()) {
		system_mode_ = req.custom_mode;
		ROS_INFO("Mode set: %s", system_mode_.c_str());
	} else {
		ROS_WARN("Mode not supported: %s", req.custom_mode.c_str());
	}

	//This service doesn't actually guarentee, rather just say the request has been made
	res.mode_sent = true;

	return true;
}

void UAVASREmulator::callback_position_target(const mavros_msgs::PositionTarget::ConstPtr& msg_in) {
	if( (msg_in->type_mask == TYPE_MASK_POS_MODE) || (msg_in->type_mask == TYPE_MASK_VEL_MODE) ) {
		position_goal_ = *msg_in;
	} else {
		ROS_WARN_THROTTLE(1.0, "Ignoring position target, type mask is not supported!");
		ROS_WARN_THROTTLE(1.0, "Available options: ");
		ROS_WARN_THROTTLE(1.0, "\tPOS: %s", (std::bitset<16>(TYPE_MASK_POS_MODE)).to_string().c_str());
		ROS_WARN_THROTTLE(1.0, "\tVEL: %s", (std::bitset<16>(TYPE_MASK_VEL_MODE)).to_string().c_str());
	}
}

double UAVASREmulator::clamp(double x, double min, double max) {
    return (x < min) ? min : ( (x > max) ? max : x );
}

};
