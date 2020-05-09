/* 	
	This node is used to follow a path.
	The movement can be controlled step by step or go autonomous
	

	Author: Jose Luis Millan Valbuena
*/

/* Libraries */
#include <math.h>
#include <unistd.h>

/* ROS libraries */
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

/* Initiate MoveBaseClient */
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class MoveWP
{
public:
	/* Class constructor. Initialization of publishers, subscribers and variables */ 
	MoveWP(): ac("move_base", true){
		std::cout<<"[MOVE_WP] Starting MoveWP."<<std::endl;

		// Subscribers
		sub_path_ = nh_.subscribe("/path", 10, &MoveWP::pathCb, this);
		sub_command_ = nh_.subscribe("/command", 10, &MoveWP::commandCb, this);

		// Publishers
		pub_next_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/next_goal", 10);
		pub_goal_status_ = nh_.advertise<std_msgs::String>("/goal_status", 10);
		
		// FrameID
		next_position_.target_pose.header.frame_id = "map";
	}

	/* Class desconstructor */ 
	~MoveWP(){
		nh_.shutdown();
		std::cout<<"\n[MOVE_WP] Destroying MoveWP. Goodbye.\n"<<std::endl;
	}

	/* Callback to reveive the command */
	void commandCb(const std_msgs::String::ConstPtr& msg){
		// "next" to go to the next point only. "auto" to do all the points without waiting commands
		if (msg->data == "next"){
			go_2_next_ = true;
			move();
			std::cout<<"[MOVE_WP] Moving once"<<std::endl;
			go_2_next_ = false;
		}else if (msg->data=="auto"){
			go_2_next_ = true;
			while(move()){
				std::cout<<"[MOVE_WP] Moving auto"<<std::endl;
			}
		}else if (msg->data=="loop"){
			go_2_next_ = true;
			int count = 0;
			while(count < 3){
				std::cout<<"Loop "<<count<<std::endl;
				while(move()){
					std::cout<<"[MOVE_WP] Moving auto"<<std::endl;
				}

				count++;
				next_ = poses_.begin();
			}
		}
		// TO DO: include patrol option: once it reaches the end, same path but backwards (ORIENTATION +pi!)
	}

	/* Callback to receive the path to follow */
	void pathCb(const nav_msgs::Path::ConstPtr& msg){
		// Store the path in private variable
		poses_ = msg->poses;
		std::cout<<"[MOVE_WP] Number of points received: "<<poses_.size()<<std::endl;
		
		// Gets first position
		next_ = poses_.begin();
	}

	
	/* Moves the robot to the position and orientates the robot for the next position*/
	int move(){
		if(next_ < poses_.end()){
			if(go_2_next_){

				// Move robot
				next_position_.target_pose.header.stamp = ros::Time::now();
				next_position_.target_pose.pose.position = next_->pose.position;	
				next_position_.target_pose.pose.orientation = next_->pose.orientation;
				pub_next_goal_.publish(*next_);
				sendGoal2Base(true);

				// Get next position
				next_++;

				/* DEPRECATED */
				// Turn robot to face next point
				/*
				if (next_!=poses_.end()){
					next_position_.target_pose.pose.orientation = next_->pose.orientation;
					sendGoal2Base(true);
				}
				*/ 
			}
		}

		if (next_ != poses_.end()){
			return 1;
		}else{
			std::cout<<"[MOVE_WP] Goal succeeded."<<std::endl;
			std_msgs::String msg_2_send;
			msg_2_send.data = "success";
			pub_goal_status_.publish(msg_2_send);
			std::cout<<"[MOVE WP] End of path reached"<<std::endl;
			return 0;
		}
		
	}

	/* Sends the goal to the MoveBaseClient. If publish_state is true, it publishes the final state */
	void sendGoal2Base(bool publish_state){
		std_msgs::String msg_2_send;

		ac.sendGoal(next_position_);

		ac.waitForResult();
		if(ac.getState()== actionlib::SimpleClientGoalState::SUCCEEDED){
			if (publish_state){
				std::cout<<"[MOVE_WP] Goal succeeded."<<std::endl;
				msg_2_send.data = "success";
				pub_goal_status_.publish(msg_2_send);
			}
		}else if(ac.getState()== actionlib::SimpleClientGoalState::PENDING){
			std::cout<<"[MOVE_WP] Goal pending."<<std::endl;
		}else if(ac.getState()== actionlib::SimpleClientGoalState::ACTIVE){
			std::cout<<"[MOVE_WP] Goal active."<<std::endl;
		}else if(ac.getState()== actionlib::SimpleClientGoalState::RECALLED){
			std::cout<<"[MOVE_WP] Goal recalled."<<std::endl;
		}else if(ac.getState()== actionlib::SimpleClientGoalState::REJECTED){
			std::cout<<"[MOVE_WP] Goal rejected."<<std::endl;
		}else if(ac.getState()== actionlib::SimpleClientGoalState::PREEMPTED){
			std::cout<<"[MOVE_WP] Goal preempeted."<<std::endl;
		}else if(ac.getState()== actionlib::SimpleClientGoalState::ABORTED){
			std::cout<<"[MOVE_WP] Goal aborted."<<std::endl;	
		}
	}

private:
	// ROS variables
	ros::NodeHandle nh_;
	ros::Subscriber sub_path_;
	ros::Subscriber sub_command_;
	ros::Publisher pub_next_goal_;
	ros::Publisher pub_goal_status_;

	MoveBaseClient ac;

	// ROS msgs
	move_base_msgs::MoveBaseGoal next_position_;

	// Auxiliary variables
	std::vector<geometry_msgs::PoseStamped> poses_;
	std::vector<geometry_msgs::PoseStamped>::iterator next_;
	bool go_2_next_;
};

int main(int argc, char **argv){
	// Initialize the node
	ros::init(argc, argv, "move_wp");

	// Initialize the class that will do everything
	MoveWP movewp;

	ros::spin();
	return 0;
}

/*	TO DO
	- pass params: frame_id, ...
*/