# move_wp
Moves the robot following a path

# Description
Follows the path received. Can go step by step or autonomous.

## ROS Version
Kinetic

## Files
launch
  - move_wp.launch

src 
  - move_wp.cpp
  
## Topics
<b>move_wp_node</b>
  - Subs
    - /path (nav_msgs::Path): path to follow
    - /command (std_msgs::String): 'next' to go to the next point. 'auto' to go point to point without waiting.
  - Pubs
    - /next_goal (geometry_msgs::PoseStamped): publish next goal to view in Rviz
    - /goal_status (std_msgs::String): publishes the status of the goal
