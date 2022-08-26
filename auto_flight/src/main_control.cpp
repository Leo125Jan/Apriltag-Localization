#include "web_control.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_control");
	ros::NodeHandle nh;

	WC web_to_px4(&nh);

	ros::Rate loop_rate(80);

	while (ros::ok())
	{
		ros::spinOnce();
		web_to_px4.Web_Command_Loop();

		loop_rate.sleep();
	}

	ros::shutdown();

	return 0;
}