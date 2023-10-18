This is a gazebo plguin that needs to be added to your URDF / gazebo files in order to make collision detection of the base work.
Please place
``` xml
  <!-- found name via: gz sdf -p turtlebot3_burger.urdf.xacro > robot.sdf -->
  <gazebo reference="base_link">
    <sensor name='collision_sensor' type='contact'>
      <update_rate>10</update_rate>
      <contact>
        <collision>base_link_collision</collision>
      </contact>
      <plugin name="collision_sensor" filename="libgazebo_ros_collision.so">
        <ros>
          <namespace>husky</namespace>
          <remapping>collision:=base_collision</remapping>
        </ros>
        <frame_name>base_footprint</frame_name>
      </plugin>
    </sensor>
  </gazebo>
``` 
in your ```gazebo.xacro``` file. The reference frame should be the global frame of your robot in simulation, which is generally ```base_footprint```. The name of the collision entry can be found via 
```bash
gz sdf -p turtlebot3_burger.urdf.xacro > robot.sdf
```
and reading the robot.sdf file to find the name of the collision object for the base link. We care about base link because that is the recommended place to define the collision object for the base robot and inertia information.

Setting the output topic and update rate are optional.

``` txt
Msg file: Contact.msg
Header header
string[] objects_hit
```
