
{
	
	gnome-terminal -- bash -c "source devel/setup.bash; roslaunch kitti_pub kitti.launch" 
}&
sleep 2s
{
	gnome-terminal -- bash -c "source devel/setup.bash; roslaunch sensor_fusion fusion.launch" 
}
