#include <ros/ros.h>
#include <uavasr_emulator/uavasr_emulator.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "uavasr_emulator");
	uavasr_emulator::UAVASREmulator uav;

	ros::spin();

	return 0;
}
