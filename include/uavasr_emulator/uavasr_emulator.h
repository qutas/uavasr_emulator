#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <uavasr_emulator/EmulatorParamsConfig.h>

#include <eigen3/Eigen/Dense>
#include <string>

namespace uavasr_emulator {

#define TYPE_MASK_POS_MODE ( mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE )
#define TYPE_MASK_VEL_MODE ( mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE )

const std::vector<std::string> mode_names = {
	"CMODE(0)",
	"MANUAL",
	"ACRO",
	"ALTCTL",
	"POSCTL",
	"OFFBOARD",
	"STABILIZED",
	"RATTITUDE",
	"AUTO_MISSION",
	"AUTO_LOITER",
	"AUTO_RTL",
	"AUTO_LAND",
	"AUTO_RTGS",
	"AUTO_READY",
	"AUTO_TAKEOFF",
};

const std::vector<int> airframe_num_motors = {
	4,
	6,
	8
};

class UAVASREmulator {

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		ros::Publisher pub_pose_;
		ros::Publisher pub_odom_;
		ros::Publisher pub_battery_;
		ros::Publisher pub_state_;
		ros::Publisher pub_target_;

		ros::Subscriber sub_target_;

		ros::ServiceServer srv_arming_;
		ros::ServiceServer srv_set_mode_;

		ros::Timer timer_pose_;
		ros::Timer timer_state_;
		ros::Timer timer_battery_;

		std::string param_frame_id_;
		std::string param_model_id_;

		double param_rate_pose_;
		double param_rate_state_;
		double param_rate_battery_;
		double param_w0_xy_;
		double param_w0_z_;
		double param_w0_psi_;
		double param_max_vel_xy_;
		double param_max_vel_z_;

		dynamic_reconfigure::Server<uavasr_emulator::EmulatorParamsConfig> dyncfg_settings_;

		mavros_msgs::PositionTarget position_goal_;

		//State Variables
		Eigen::Vector3d p_;
		Eigen::Vector3d v_;
		double yaw_;


		bool system_armed_;
		std::string system_mode_;
		double param_auto_disarm_;
		double height_max_;

		tf2_ros::TransformBroadcaster tfbr_;

	public:
		UAVASREmulator( void );
		~UAVASREmulator( void );

	private:
		void callback_cfg_settings( uavasr_emulator::EmulatorParamsConfig &config, uint32_t level );

		void callback_pose(const ros::TimerEvent& e);
		void callback_state(const ros::TimerEvent& e);
		void callback_battery(const ros::TimerEvent& e);

		void callback_position_target(const mavros_msgs::PositionTarget::ConstPtr& msg_in);

		bool callback_set_mode(mavros_msgs::SetMode::Request &req, mavros_msgs::SetMode::Response &res);
		bool callback_arming(mavros_msgs::CommandBool::Request &req, mavros_msgs::CommandBool::Response &res);

		double yaw_error_shortest_path(const double y_sp, const double y);

		double clamp(double x, double min, double max);
};

};
