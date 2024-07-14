#!/bin/bash


NUM_ROBOTS=$1;
x_max=5;
y_max=5;

cat <<EOF > /home/fachrurrozi18/swarm_ws/src/swarm/launch/dynamic_swarm.launch
<launch>
  <include file="\$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="\$(find turtlebot3_gazebo)/worlds/empty.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
EOF

for i in $(seq 1 $NUM_ROBOTS); do
  cat <<EOF >> /home/fachrurrozi18/swarm_ws/src/swarm/launch/dynamic_swarm.launch
  <group ns="tb3_$i">
    <param name="robot_description" command="\$(find xacro)/xacro '\$(find turtlebot3_description)/urdf/turtlebot3_burger.urdf.xacro'"/>
    <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" output="screen" args="-urdf -model tb3_$i -x $((-$x_max+RANDOM%(2*$x_max))).$((RANDOM%999)) -y $((-$y_max+RANDOM%(2*$y_max))).$((RANDOM%999)) -z 0.5 -param robot_description"/>
  </group>
EOF
done

echo "</launch>" >> /home/fachrurrozi18/swarm_ws/src/swarm/launch/dynamic_swarm.launch

roslaunch swarm dynamic_swarm.launch