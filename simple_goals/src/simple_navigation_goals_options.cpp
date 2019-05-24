#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include "sound_play/sound_play.h"

using namespace std;

std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);

/** declare the coordinates of interest **/
double xCafe = -3.75;
double yCafe = -0.45;
double xOffice1 = 1.935 ;
double yOffice1 = -0.849;
double xOffice2 = -2.461;
double yOffice2 = -0.013;
double xOffice3 = 35.20 ;
double yOffice3 = 13.50;

bool goalReached = false;
 int main(int argc, char** argv){
   ros::init(argc, argv, "map_navigation_node");
   ros::NodeHandle n;
   //sound_play::SoundClient sc;
   ros::spinOnce();
   //path_to_sounds = "/home/ros/catkin_ws/src/gaitech_edu/src/sounds/";
   
   char choice;
   
   do{
      cout << "Please select a place you would like to navigate to\n0. upper left\n1.upper right\n2.bottom";
      cin >> choice;
      if (choice == '0'){
         goalReached = moveToGoal(xCafe, yCafe);
      }else if (choice == '1'){
         goalReached = moveToGoal(xOffice1, yOffice1);
      }else if (choice == '2'){
         goalReached = moveToGoal(xOffice2, yOffice2);
      }else if (choice == '3'){
         goalReached = moveToGoal(xOffice3, yOffice3);
      }
      if (choice!='q'){
         if (goalReached){
            ROS_INFO("Congratulations!");
            ros::spinOnce();
            //sc.playWave(path_to_sounds+"ship_bell.wav");
            ros::spinOnce();

         }else{
            ROS_INFO("Hard Luck!");
            //sc.playWave(path_to_sounds+"short_buzzer.wav");
         }
      }
   }while(choice !='q');
   return 0;
}
bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a 
ActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }

}
