
# New features

1. Safety checks on the motion solution. If they fail, the motion optimization
   will fail and the trajectory query tool sill never be initialized. Verifies 
   that joint limits are satisfied and that velocities and accelerations are 
   within a nominal range.
2. Motion optimization is more robust. The suite of demos and test runs is more
   comprehensive, varying target locations as well as differing behavior types.
3. Switching between moving over the object vs moving around the object is now
   handled through modeling of the Riemannian geometry of the workspace within
   the RieMO (Riemannian Motion Optimization) framework. The Riemannian
   geometry of the workspace defines the meaning of geodesic (shortest path) in
   the workspace; when biasing toward moving over the table we stretch the
   workspace so that shortest paths tend to pass over the object, and when
   biasing toward moving around the object we stretch the workspace so that
   shortest paths tend to go around. Both obstacle_linearization_constraint_csv
   and passthrough_constraint_csv are still available in the API, they're just
   unnecessary now for the purposes of switching between moving over vs around.
4. Added approach shaping wherein the robot explicitly approaches the target
   from the approach direction (rather than just achieving that orientation at
   the end). During the approach the robot opens the hand, and then closes it
   at the end to establish a grasp.
5. Reoriented the robot so that the x-axis of the root frame points forward
   into the primary area of the workspace. Previously we had the y-axis
   pointing forward.  This new orientation better matches the range of the
   first joint, which has the robot pointing forward at its center
   configuration of q1 = 0.


# Setting up and building

System requirements: Ubuntu 14.04 and ROS Indigo

Run the following:

    mkdir lula; cd lula
    git clone https://github.com/lularobotics/nw_init.git
    ./nw_init/init.sh

Follow the instructions:
1. Set the credientials to the ones we sent you by email.
2. Say yes when it asks to update the binary. If you've already run this step,
   the update will be very fast because the image will already be cached on
   your local machine. It's a good idea to always run this step.
3. Say yes to setting up the lularobotics_ws workspace.
4. If there is a problem with the build, you can always rebuild manually by
   going into the lularobotics_ws workspace and running

       catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
   
   In general, when changing any of the client code (see the note below about
   playing with speed modulation in the execution), you can rebuild manually
   this way.


# How to run the demo(s)

##### Brief:

In general, startup and shutdown of the RieMO server can be handled though the
following commands:

##### Startup RieMO server:

    roslaunch nw_motion_optimization riemo_mico_server_with_robot_emulator.launch

##### Shutdown RieMO server:

    rosrun nw_motion_optimization riemo_mico_server_shutdown.sh

Once the server is running one can start planning actions and query for trajectories
using the 

    riemo_move_action/action/Plan.action 
  
interface. Example usage is given in 

    nw_mico_client/scripts/nw_mico_simple_move_client.py 

See the demos for specific examples of the usage.

-------------------------------------------------------------------------------

#### More details

There are four nodes that need to be started in order to setup the system
1. Rviz with the /robot_description parameter
2. The robot emulator: this node serves to substitute for the real robot in
   simulation.  It keeps track of and publishes the current state, and accepts
   joint trajectory messages to emulate their execution. Sends transforms to
   Rviz for visualization of the movements.
3. The robot visualizer: Subscribes to the /joint_states topic and publishes
   transforms to rviz. The /joint_states come from either the emulator or ROS
   control on the real robot.
4. Motion optimization service: Creates the planning action service and the 
   trajectory query service.

And finally, once those are setup, motion optimization requests can be sent to
the robot using the Plan action API discussed below. A simple example of how to 
use the API is given in 

  c++:    nw_mico_client/src/nw_mico_client/nw_mico_simple_move_client_main.cpp
  python: nw_mico_client/scripts/nw_mico_simple_move_client.py

Both clients implement basically the same functionality. The cpp client demos
more meticulously slowing down and speeding up the execution, but the demos
listed below use the python script for its simplicity.

To start these nodes up and run the client, run the following (making 
sure that the workspace's devel/setup.bash script is sourced for each:

-------------------------------------------------------------------------------

### Explicit instructions for running the demos:

#### Demo: Comprehensive

##### In terminal 1 
When using the emulator

    roslaunch nw_motion_optimization riemo_mico_server_with_robot_emulator.launch
    
For real robot: 

    launch nw_motion_optimization riemo_mico_server.launch
  
##### In terminal 2

    rosrun nw_mico_client run_basic_mico_demos
    
Or, for a more comprehensive version: 

    rosrun nw_mico_client run_comprehensive_mico_demos

##### Shutdown when done

    rosrun nw_motion_optimization riemo_mico_server_shutdown.sh

#### Demos:
- run_basic_mico_demos: Runs each of the features back and forth across the
  spherical object just once.
- run_comprehensive_mico_demos: Runs a more comprehensive set of demos. Uses
  the same ordering as before, but uses a series of 8 target points for each
  configuration.
- Configurations for each demo as given in the config/demos directory of nw_mico_client

#### Details about the ordering.

Conceptually, the demos are ordered as follows:

    for approach_type in ["use_approach", "no_approach"]:
      for upright_type in ["end_only", "whole_traj", "none"]":
        for behavioral_type in ["default", "around", "over"]:
          for target in target_list:
            run_demo_with(appraoch_type, upright_type, behavioral_type, target)
          done
        done
      done
    done

The demos run everything with the approach shaping first, and then everything
again without the approach shaping. For each upright constraint type it runs a
series of trials with different behavior types, starting with none (default),
then proceeding to moving around the object, and then moving over the object.
It first exercises end-only upright constraints to make transitions from no
end-effector constraints safe, then applies the upright constraint to the
entire trajectory, and finally removes it entirely, running the series of
behavior types for each. And finally, for each individual trial type, it runs
the trial on a series of target points moving back and forth across the
obstacle in the center.

### Demo: Playground

#### In terminal 1 
When using the emulator:

    roslaunch nw_motion_optimization riemo_mico_server_with_robot_emulator.launch
    
For real robot: 

    launch nw_motion_optimization riemo_mico_server.launch

#### In terminal 2
Now we can run the client to make planning requests. In the following,

    # <x>, <y>, <z> specifies a target location. The specified yaml configuration file relative
    # to the nw_mico_client package and specifies the behavioral information needed to fill
    # in the motion optimizatino action request.
    rosrun nw_mico_client nw_mico_simple_move_client.py config/mico_move_task_config.yaml <x> <y> <z>

To play with the settings of the config:

    roscd nw_mico_client/config
    vim mico_move_task_config.yaml

Also, we can place a sphere in the environment by executing the following command a 
convenient configuration is (.5, .0, .1) with a radius of .15.

    rosrun nw_mico_client set_obstacle_parameters <x> <y> <z> <radius>

#### Shutdown when done

  rosrun nw_motion_optimization riemo_mico_server_shutdown.sh

### Launching the server side components manually without the launch file:
##### In terminal 1

    roslaunch nw_mico_client mico_rviz_only.launch

##### In terminal 2
 
    rosrun nw_motion_optimization start_motion_optimization_emulator.sh

##### In terminal 3

    rosrun nw_motion_optimization start_motion_optimization_visualizer.sh

##### In terminal 4

    rosrun nw_motion_optimization start_motion_optimization_service.sh

#### To stop 2 through 4 these, run (substituting emulator, visualizer, service for <component>):

    rosrun nw_motion_optimization stop_motion_optimization_<component>.sh


### Troubleshooting

- When rviz comes up, some times the main screen is black. This seems to be a
  race condition within rviz, itself. Closing it down and restarting it usually
  works.
- When rviz starts up, and before the robot emulator has been started, the
  robot has no registered transforms so it is just a bundle of unrendered parts
  at the origin. This goes away immediately once the robot emulator has
  started.
- The code uses c++11. If you're not familiar with it, you might find the auto
  keyword confusing.  It simply tells the compiler to infer the type from the
  right hand side of the equals sign.


### How to shutdown the riemo planning server

The planning server and robot visualization and  emulation require an explicit shutdown.
In order to do that launch the shutdown procedure as follows:

In any terminal

    roslaunch nw_motion_optimization shutdown_riemo_mico_server_with_robot_emulator.launch


## The basic API

### ROS topics
- /joint_states : the robot emulator publishes the current joint states on this
  topic as sensor_msgs::JointState messages.  The motion optimization service
  consumes these messages to know where the robot currently is when it starts
  planning and whenever it is queried for a trajectory once the LQR is ready.
- /joint_trajectory : The trajectory query service publishes on this topic.
  The robot emulator consumes these messages and emulates movement along the
  trajectory while sending visualization transforms to Rviz.


See the demo client for an example of how to use the API:

    nw_mico_client/src/nw_mico_client/nw_mico_simple_move_client_main.cpp

### Basic decomposition:
- The client makes a planning request to the planning action service
- Once motion optimization is complete, the client is notified, and the
  trajectory query service is ready. The trajectory query service is an
  interface to querying trajectories from the LQR resulting from the motion
  optimization.
- Trajectories can be queried with differing time-dilations to speed up or slow
  down the trajectory.

### Setting the request: 
- Basic API specified through: riemo_move_action/action/Plan.action
- Here's a detailed breakdown of the request's fields taken from the action definition:

      # 3D target position
      geometry_msgs/Point target
      
      # Behavior type of "over" makes the robot go over the object; behavior type of
      # "around" makes it go around the object.
      string behavioral_type
      string BEHAVIORAL_TYPE_DEFAULT=default
      string BEHAVIORAL_TYPE_OVER=over
      string BEHAVIORAL_TYPE_AROUND=around
      
      # Specifies the orientation the end-effector should have to be considered
      # "upright". Must be orthogonal to the approach_constraint_csv. Constrains the
      # y-axis of the end-effector. Application of this constraint are governed by
      # use_upright_orientation_constraint{_end_only}.
      string upright_constraint_direction_csv
      
      # Set to true to include an orientation constraint to keep the hand upright
      # throughout the entire motion.
      bool use_upright_orientation_constraint
      
      # Set to true to include an orientation constraint to keep the hand upright
      # only at the final configuration.
      bool use_upright_orientation_constraint_end_only
      
      # Note on format: in the following <v{x,y,z}> denotes a vector and <pt{x,y,z}>
      # denotes a point.
      
      # Specify an approach constraint. This constraint is a vector, specified as a comma
      # separated string of three numbers, specifying the direction that the end-effector should
      # approach the target from at the end of the motion.
      string approach_constraint_csv # Format: <vx>,<vy>,<vz>
      
      # When true, approaches the target *from* the approach direction and opens and
      # closes the grippers to establish a grasp.
      bool shape_approach
      
      # This CSV gives a ray pointing from the the center of the sphere to the
      # surface where a linearization constraint should be added. A linearization
      # constraint is a linear function whose zero set is offset by .03 m radially
      # from the tangent to the surface and increases away from the sphere. The
      # constraint is applied t_fraction of the way through the trajectory and it
      # enforces that the end-effector be on the position side of the plane at that
      # point.
      string obstacle_linearization_constraint_csv # Format: <vx>,<vy>,<vz>,<t_fraction>
      
      # The passthrough constraint is a more restrictive variant of a constraint that
      # constrains the behavior of the robot as it moves from point to point. It gives
      # a full box at (<ptx>,<pty>,<ptz>) with 1/2 side length (radius) of <radius>. 
      # It is again applied t_fraction of the way through the trajectory.
      string passthrough_constraint_csv # Format: <ptx>,<pty>,<ptz>,<radius>,<t_fraction>



#### Placing and removing spherical obstacles:

    rosrun nw_mico_client set_obstacle_parameters <x> <y> <z> <radius>
    rosrun nw_mico_client set_obstacle_parameters clear

##### Example

    # Place a nominally sized sphere of radius .15 m at pt = (0, .5, .1).
    rosrun nw_mico_client set_obstacle_parameters 0 .5 .1 .15

    # Now remove the sphere
    rosrun nw_mico_client set_obstacle_parameters clear


### How to play around with the system

Moving the obstacle around: The sphere can be moved around and even removed 
as specified in the section above.

Modifying the constraints: The easiest way to play around with the planner
is to use run_riemo_move_mico_playground. The user can turn on and off various
subsets of constraints by setting and unsetting the shell script variables.


### Modulating the speed of execution in the demos on the fly

Uncomment the line `"#define MODULATE_SPEED"` in 

    nw_mico_client/nw_mico_simple_move_client_main.cpp

and rebuild the client to see an example of repeatedly slowing down and
speeding up the motion mid-execution.
