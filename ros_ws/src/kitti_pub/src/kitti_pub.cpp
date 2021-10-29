#include "kitti_pub/kitti_data_utils.hpp"
int main(int argc, char * argv[]) {
    // int frame = 1;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiPublisher>());
    rclcpp::shutdown();
    return 0;
}
