# Requery trajectory update Mar 2, 2016

1. Bug fixes for requerying of the trajectory. Requery trajectory using
  ```
   rosrun nw_mico_client nw_mico_requery_trajectory.py <dilation_factor>
  ```
2. Temporarily removed finger joint command transformation. Now publishes 
   on /joint_trajectory only (no /joint_trajectory_raw topic), and the emulator
   listens to that topic. Publishes joint angles (and velocties / accelerations)
   for those joints as well.

# New features for update URDF

1. Uses the URDF defined in nw_mico/mico-modified-working.urdf rather than the
   kinova-ros URDF. This URDF is a slightly modified version of the NW URDF,
   most notably the joint limits have been modified to prevent self-collisions.
   The original values are commented out, and comments describe the rationale
   behind the changes.
2. The client `nw_mico_client/scripts/nw_mico_simple_move_client.py` now
   queries the trajectory rather than "broadcasting" it, so that it can
   transform the finger dimensions into finger commands in the range [0, 6000]
   before publishing. The upper joint limit for the fingers is set to .8 which
   is 2/3s of the maximal value of 1.2 so that the maximum finger command when
   scaled by the client ends up being 4000. It publishes the transformed
   trajectory on `/joint_trajectory` and the raw trajectory (with joint angles
   for the finger dimensions) on `/joint_trajectory_raw`. The robot emulator
   now listens to the `/joint_trajectory_raw` topic so that it can visualize
   the raw finger joint angles.
3. Added "pickup" and "place" options in Plan.action: When "shape_approach" is
   not set, "pickup" and "place", when true, will enable specialized pickup and
   place motions at the beginning and end of the trajectory, respectively. They
   can be specified independently or in combination. Additionally, the gripper
   remains closed by default unless "shape_approach" is true, in which case it
   opens and closes as usual to establish a grasp. When "shape_approach" is
   true, both "pickup" and "place" are ignored, because it's assumed that a
   grasp has not yet been established. The demos that don't use
   "shape_approach" now use both the "pickup" and "place" options.
    

# Prev new features 

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


