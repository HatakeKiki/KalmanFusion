filename=`date +%Y_%m_%d`
directory="./src/sensor_fusion"
cd ${directory}
if [ ! -d ${filename} ];then
  echo ${filename}"不存在， 正在创建文件夹"
else
  echo "文件已存在， 将删除后重建"
  rm -r ${filename}
fi
if [ ! -f ${filename}.zip ];then
  rm -f ${filename}.zip
  echo "压缩文件已存在，将删除后重建"
fi
. mkdir.sh

cd ..
cd ..

source /opt/ros/dashing/setup.bash
. install/setup.bash
ros2 launch sensor_fusion sensor_fusion_launch.py
cd ${directory}
zip -r ${filename}.zip ${filename}

cd ..
cd ..
