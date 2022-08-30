# BD1 Self Balancer

<p align = "justify">
This package contains the code for navigation tasks for the holonomic drive robot.</p>

Install Dependencies to ensure all packages are installed.

        rosdep install --from-paths ./ -i -y --rosdistro humble --ignore-src

Launch the arena to test the code using the below-mentioned command.

        ros2 launch hdr_navigation cafe_world.launch.py
        ros2 launch hdr_navigation smalltown_world.launch.py

Run the keyboard controller to drive the duke bot.

        ros2 run hdr_navigation keyboard_control

<div style="text" align="center">
    <img src="https://github.com/hari-vickey/ROS2-HDR/blob/main/images/test_hdr_world1.png" />
</div>
<div style="text" align="center">
    <img src="https://github.com/hari-vickey/ROS2-HDR/blob/main/images/test_hdr_world2.png" />
</div>
