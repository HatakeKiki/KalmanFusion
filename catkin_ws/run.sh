
{
	gnome-terminal -- bash -c "cd ./src/kitti_pub; ./kitti_compile.sh" 
}&
sleep 2s
{
	gnome-terminal -- bash -c "cd ./src/darknet_ros; ./darknet_compile.sh" 
}
