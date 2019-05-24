#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>


#define SEUNGRI 11
#define TAEYANG 12
#define TOP 13

using namespace std;

std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal, string name);
void objectCallback();

/** declare the coordinates of interest **/
//right map
/**
double xCafe = 3.436;
double yCafe = -0.475;
double xOffice1 = 3.362 ;
double yOffice1 = -1.202;
double xOffice2 = 2.447;
double yOffice2 = -1.287;
double xOri = 1.362 ;
double yOri = -0.675;
**/

//left map
double xCafe = 2.204;
double yCafe = -0.199;
double xOffice1 = 3.362 ;
double yOffice1 = -1.202;
double xOffice2 = 2.447;
double yOffice2 = -1.287;
double xOri = 0.222 ;
double yOri = -1.058;
bool goalReached = false;

int id=0;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
   if (object->data.size() > 0)
   {
      id = object->data[0];

      switch (id)
      {
      case SEUNGRI:
         goalReached = moveToGoal(xCafe, yCafe, "Seungri");
         break;
      case TAEYANG:
         goalReached = moveToGoal(xOffice1, yOffice1, "Taeyang");
         break;
      case TOP:
         goalReached = moveToGoal(xOffice2, yOffice2, "T.O.P");
         break;
      default: // other object
         ROS_INFO("The recognized object is not define in the coding.");
      }
      //go back original position
      moveToGoal(xOri, yOri, "the place i born");  
   }
   else
   {
      // No object detected
      ROS_INFO("I have detected nothing :(");
   }
}

 int main(int argc, char** argv){
   ros::init(argc, argv, "map_navigation_node");
   ros::NodeHandle n;
   ros::Rate loop_rate(50);
   //ros::Publisher initial_pose_pub = n.advertise<std_msgs::String>("initialpose", 1000);   

   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

   while (ros::ok())
   {
      ROS_INFO("Listening to recognized object id");
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}
bool moveToGoal(double xGoal, double yGoal, string name){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
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

   std_msgs::String s;
   s.data = name;

   ROS_INFO("Moving to goal location to find %s...", s.data.c_str());
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("I have reached the destination");

      //add new code here to rotate
      //if found, refer the bookmarked coding(approaching), else, return to original place.
      //ROS_INFO("Rotating to find" + name); (override the method, add one more argument name)

      ros::Duration(2).sleep();	//after I found any bigbang member, I will stand still to salute for 2 seconds

      return true;
   }
   else{
      ROS_INFO("I failed to reach the destination");
      return false;
   }



}
