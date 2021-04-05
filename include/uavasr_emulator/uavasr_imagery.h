#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>
#include <uavasr_emulator/ImageryParamsConfig.h>

#include <eigen3/Eigen/Dense>
#include <string>

#include <opencv2/core.hpp>

namespace uavasr_emulator {

class UAVASRImagery {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		dynamic_reconfigure::Server<uavasr_emulator::ImageryParamsConfig> dyncfg_settings_;

		image_transport::ImageTransport it_;
		image_transport::Publisher pub_image_;

		ros::Subscriber sub_pose_;
		ros::Timer timer_image_;

		std::string param_frame_id_;
		std::string param_floor_texture_;
		double param_floor_px_m_;
		double param_framerate_;
		double param_field_of_view_;
		int param_resolution_width_;
		int param_resolution_height_;

		bool update_perspective_;
		geometry_msgs::PoseStamped pose_;
		sensor_msgs::ImagePtr img_;
		cv::Mat texture_;

	public:
		UAVASRImagery( void );
		~UAVASRImagery( void );

	private:
		void callback_cfg_settings( uavasr_emulator::ImageryParamsConfig &config, uint32_t level );

		void callback_image(const ros::TimerEvent& e);
		void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg_in);

		void load_floor_texture(const std::string filename);
		cv::Mat calc_perspective(const geometry_msgs::Pose pose);
		void calc_camera_output(const cv::Mat perspective);
};

};
