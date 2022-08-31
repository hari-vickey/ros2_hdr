# BD1 Self Balancer

<p align = "justify">
This package contains the code for navigation tasks for the holonomic drive robot.</p>

Install Dependencies to ensure all packages are installed.

        rosdep install --from-paths ./ -i -y --rosdistro humble --ignore-src

Launch the arena to test the code using the below-mentioned command.

        ros2 launch hdr_navigation ddr_bot_world.launch.py # defaults cafe.world
        ros2 launch hdr_navigation duke_bot_world.launch.py world:=smalltown.world

Run the keyboard controller to drive the ddr bot.

        ros2 run teleop_twist_keyboard teleop_twist_keyboard

Run the keyboard controller to drive the duke bot.

        ros2 run hdr_navigation keyboard_control

<div style="text" align="center">
    <img src="https://github.com/hari-vickey/ROS2-HDR/blob/main/images/test_hdr_world1.png" />
</div>
<div style="text" align="center">
    <img src="https://github.com/hari-vickey/ROS2-HDR/blob/main/images/test_hdr_world2.png" />
</div>
