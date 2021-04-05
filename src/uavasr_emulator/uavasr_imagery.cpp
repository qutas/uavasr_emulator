#include <ros/ros.h>

#include <uavasr_emulator/uavasr_imagery.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>

#include <string>
#include <eigen3/Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace uavasr_emulator {


UAVASRImagery::UAVASRImagery() :
	nh_(),
	nhp_("~"),
	param_frame_id_("map"),
	param_floor_texture_("uav"),
	param_floor_px_m_(256),
	param_framerate_(10.0),
	param_field_of_view_(60.0),
	param_resolution_width_(640),
	param_resolution_height_(480),
	update_perspective_(false),
	dyncfg_settings_(ros::NodeHandle(nhp_)),
	it_(nhp_) {

	dyncfg_settings_.setCallback(boost::bind(&UAVASRImagery::callback_cfg_settings, this, _1, _2));

	//Parameters
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("floor_texture", param_floor_texture_, param_floor_texture_);
	nhp_.param("floor_px_m", param_floor_px_m_, param_floor_px_m_);
	nhp_.param("framerate", param_framerate_, param_framerate_);
	nhp_.param("resolution_width", param_resolution_width_, param_resolution_width_);
	nhp_.param("resolution_height", param_resolution_height_, param_resolution_height_);

	//Floor Texture
	load_floor_texture(param_floor_texture_);

	if( ros::ok() ) {
		//Perpare output image
		calc_camera_output(cv::Mat());	//Pass an empty matrix to get an intial black view

		pub_image_ = it_.advertise("image_raw", 1);
		sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>( "pose", 10, &UAVASRImagery::callback_pose, this );
		timer_image_ = nhp_.createTimer(ros::Duration(1.0/param_framerate_), &UAVASRImagery::callback_image, this );

		ROS_INFO("UAVASR imagery started!");
	}
}

UAVASRImagery::~UAVASRImagery() {
}


void UAVASRImagery::callback_cfg_settings( uavasr_emulator::ImageryParamsConfig &config, uint32_t level ) {
	param_framerate_ = config.framerate;
	param_field_of_view_ = config.field_of_view;
	param_resolution_width_ = config.resolution_width;
	param_resolution_height_ = config.resolution_height;

	timer_image_.setPeriod(ros::Duration(1.0/param_framerate_));
}

void UAVASRImagery::callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
	pose_ = *msg_in;
	update_perspective_ = true;
}

void UAVASRImagery::callback_image(const ros::TimerEvent& e) {
	if(update_perspective_) {
		calc_camera_output(calc_perspective(pose_.pose));
		update_perspective_ = false;
	}

	img_->header.stamp = ros::Time::now();
	pub_image_.publish(img_);
}

void UAVASRImagery::load_floor_texture(const std::string filename) {
    texture_ = cv::imread(filename, cv::IMREAD_COLOR);

	if(texture_.empty()) {
		ROS_ERROR("Could not load floor texture at: %s", filename.c_str());
		ros::shutdown();
	}
}

cv::Mat UAVASRImagery::calc_perspective(const geometry_msgs::Pose pose) {
	//Definitions of the perspective points in clockwise order
	//Destination is simply the camera frame
	cv::Point2f src[4];
	cv::Point2f dst[4] = {
		cv::Point2f(0, 0),
		cv::Point2f(0, param_resolution_height_),
		cv::Point2f(param_resolution_width_, param_resolution_height_),
		cv::Point2f(param_resolution_width_, 0)
	};

	//Calculate source points
	bool valid = true;

	//TODO

	if(valid) {
		//Calculate and return perspective matrix
		return cv::getPerspectiveTransform(src, dst);
	} else {
		return cv::Mat();	//Return no perspective if it is not possible
	}
}

void UAVASRImagery::calc_camera_output(const cv::Mat perspective) {
	cv_bridge::CvImage cv_img;
	std_msgs::Header header;
	header.frame_id = param_frame_id_;
	cv::Mat img = cv::Mat(param_resolution_height_, param_resolution_width_, CV_8UC3);

	if( !perspective.empty() ) {
		cv::Size img_size(param_resolution_width_, param_resolution_height_);
		cv::warpPerspective(texture_, img, perspective, img_size);
	} //else, don't have a valid perspective, show black

	cv_img = cv_bridge::CvImage(header, "bgr8", img);
	img_ = cv_img.toImageMsg();
}


};

