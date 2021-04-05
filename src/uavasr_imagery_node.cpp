#include <ros/ros.h>
#include <uavasr_emulator/uavasr_imagery.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "uavasr_imagery");
	uavasr_emulator::UAVASRImagery img;

	ros::spin();

	return 0;
}
