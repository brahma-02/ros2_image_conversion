# ros2_image_conversion

Create a workspace and src file:
  mkdir -p ros2_ws/src

cd ros2_ws/src 
Download the "image_conversion_pkg" from the git repo and paste inside the src file

Build the package 
cd ..
colcon build 
source install/setup.bash 

Launch:
  ros2 launch image_conversion_pkg image_conversion_launch.py

Set the mode :
  ros2 service call /set_image_mode std_srvs/srv/SetBool "{data: true}"  # Grayscale
  ros2 service call /set_image_mode std_srvs/srv/SetBool "{data: false}" # Color

View image :
  ros2 run rqt_image_view rqt_image_view
