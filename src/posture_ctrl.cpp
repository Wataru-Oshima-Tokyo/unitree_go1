 #include <ros/ros.h>



 // Include CvBridge, Image Transport, Image msg
 #include "std_msgs/String.h"
 #include "std_msgs/Bool.h"
 #include "geometry_msgs/Twist.h"
 #include "unitree_a1/actions.h"
 #include "std_srvs/Empty.h"
 
 #include <vector>


 class POSTURE{
   //  private:

    public:
      ros::NodeHandle nh;
      ros::ServiceServer action_start;
      virtual bool actions_srv(unitree_a1::actions::Request& req, unitree_a1::actions::Response& res);
      ros::Publisher action_execution, action_cmd;
      

      const std::string ACTION_SERVICE_START = "/action/start";
      const std::string ACTION_CMD_TOPIC = "/cmd_vel_posture";
      const std::string ACTION_EXE_TOPIC = "cmd_vel_executing";
      struct timespec start, stop;
      double fstart, fstop;
      POSTURE();
      ~POSTURE();
 };
   POSTURE::POSTURE(){}
   POSTURE::~POSTURE(){}

   bool POSTURE::actions_srv(unitree_a1::actions::Request& req, unitree_a1::actions::Response& res){
      geometry_msgs::Twist cmd;
      std_msgs::Bool exe;
      exe.data = true;
      if(req.action ==0){
         //pass through
      }else if (req.action == 1){
         //look up
         cmd.linear.x = 1;
      }else if (req.action == 2){
         //look down
         cmd.linear.x = -1;
      }else if (req.action == 3){
         //look left
         cmd.angular.z = 1;
      }else if (req.action == 4){
         //look right
         cmd.angular.z = -1;
      }else if (req.action ==5){
         //talk
      }

      clock_gettime(CLOCK_MONOTONIC, &start); fstart=(double)start.tv_sec + ((double)start.tv_nsec/1000000000.0);
      clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
      action_execution.publish(exe);
      while((fstop-fstart) < req.duration){
         action_cmd.publish(cmd);
         clock_gettime(CLOCK_MONOTONIC, &stop); fstop=(double)stop.tv_sec + ((double)stop.tv_nsec/1000000000.0);
      }
      exe.data = false;
      action_execution.publish(exe);
      
   }


 int main (int argc, char** argv){
   ros::init(argc, argv, "postrue_services");
   POSTURE ps;
   ps.action_start = ps.nh.advertiseService(ps.ACTION_SERVICE_START, &POSTURE::actions_srv, &ps);
   ps.action_cmd = ps.nh.advertise<geometry_msgs::Twist>(ps.ACTION_CMD_TOPIC,1000);
   ps.action_execution = ps.nh.advertise<std_msgs::Bool>(ps.ACTION_EXE_TOPIC,1000);
   return 0;
 }

