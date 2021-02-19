// #include <cstdio>
#include "epos_base.hpp"

#define NAME_SPACE "epos_single"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<epos_base>(NAME_SPACE);
	
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
