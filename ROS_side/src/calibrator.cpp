#include <ros/ros.h>
#include "std_msgs/String.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float64MultiArray.h"

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <exception>
//#include <conio.h>
#include <math.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
 #include <arpa/inet.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
 #include <boost/asio.hpp>


#define BUFLEN 1024  //Max length of buffer
#define PORT "12358"   //The port on which to listen for incoming data
#define UDP_SERVER_IP "172.31.1.147"//define the udp server ip address  //"127.0.0.1" "172.31.1.147"/

using boost::asio::ip::udp;

enum { max_length = 1024 };


using namespace std;

std::vector<cv::Point3d> pts_marker;
std::vector<cv::Point3d> pts_robot;
std::vector< std::vector<double> > pose_marker;
std::vector< std::vector<double> > pose_robot;
double markerX;
double markerY;
double markerZ;
double markerQx;
double markerQy;
double markerQz;
double markerQw;
bool markerValid=false;
void die(char *s)
{
    perror(s);
    exit(1);
}

string double2str(double d)
{
  std::ostringstream d2str;
  d2str<<d;
  return d2str.str();
}

string constructCorrsPtsStr(cv::Point3d kinectPoint, cv::Point3d robotPoint, std::vector<double> poseMarker, std::vector<double> poseRobot)
{
  return double2str(kinectPoint.x)+","+double2str(kinectPoint.y)+","+double2str(kinectPoint.z)+","+
         double2str(poseMarker[0])+","+double2str(poseMarker[1])+","+double2str(poseMarker[2])+","+double2str(poseMarker[3])+","+
         double2str(robotPoint.x)+","+double2str(robotPoint.y)+","+double2str(robotPoint.z)+","+
	double2str(poseRobot[0])+","+double2str(poseRobot[1])+","+double2str(poseRobot[2])+","+double2str(poseRobot[3])+
	"\r\n";
}

 std::vector<float> convert2Float(std::vector<double> v)
 {
   std::vector<float> t;
   for(int i=0;i<v.size();i++)
     t.push_back((float)v[i]);
   return t;
}

void printOutStdVector(std::vector<double> v)
{
  //cout<<"print out v:"<<endl;
  for(int i=0;i<v.size();i++)
  {
    cout<<v.at(i)<<"  ";
  }
  cout<<endl;
}

bool packageValid(string inputStr)
{
   if(inputStr.find("@l@") != std::string::npos)
     return true;
   else
     return false;
}

 std::vector< double > fromString2Array(string inStr)
{
    std::vector< double > vd;
    string buf; // Have a buffer string
    stringstream ss(inStr); // Insert the string into a stream
    while (ss >> buf)
        vd.push_back(atof(buf.c_str()));
    return vd;
}

std::vector< string > fromString2ArrayStr(string inStr)
{
   std::vector< string > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back((buf.c_str()));
   return vd;
}

void ar_pose_marker_callback(ar_track_alvar_msgs::AlvarMarkers req)
 {
   size_t ll=sizeof(req.markers[0]);

   int sizemarker=req.markers.size();

   if(!req.markers.empty())
   {
      for(int i=0;i<sizemarker;i++)
      {
	ar_track_alvar_msgs::AlvarMarker marker=req.markers[i];
	int id=marker.id;
	int con=marker.confidence;
	geometry_msgs::Pose pose=marker.pose.pose;
	geometry_msgs::Point position=pose.position;
	// tf::Quaternion q(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w);
	markerValid=false;
	switch(id)
	{
	  case 3:
	    markerX=position.x;
	    markerY=position.y;
	    markerZ=position.z;
	    markerQx=pose.orientation.x;
	    markerQy=pose.orientation.y;
	    markerQz=pose.orientation.z;
	    markerQw=pose.orientation.w;

	    markerValid=true;
	    break;
	}
      }
   }
 }

void add_marker_position()
{
  std::cout << "get marker posotion:  "<<markerX<<","<<markerY<<","<<markerZ<<","<<markerQx<<","<<markerQy<<","<<markerQz<<","<<markerQw<<endl;
  cv::Point3d currentMarker(markerX,markerY,markerZ);
  pts_marker.push_back(currentMarker);
  std::vector<double> currentMarkerPose;
  currentMarkerPose.push_back(markerQx);
  currentMarkerPose.push_back(markerQy);
  currentMarkerPose.push_back(markerQz);
  currentMarkerPose.push_back(markerQw);
  pose_marker.push_back(currentMarkerPose);
}

void resolvRobotPosition()
{
  char in_reply[max_length];
  udp::endpoint sender_endpoint;
  size_t reply_length = s.receive_from(boost::asio::buffer(in_reply, max_length), sender_endpoint);
  std::cout << "Reply is: "; std::cout.write(in_reply, reply_length); std::cout << "\n";

  if(reply_length>0)
  {
    char readfrom[reply_length];
    for(int i=0;i<reply_length;i++)
    readfrom[i]=in_reply[i];
    string inStr(readfrom);
    std::cout << "msg received:  "+inStr<<endl;
    if (packageValid(inStr))//get the robot position
    {
        int endIndex=str.find("@l@");
        string strTemp=inStr.substr(0,endIndex);
        //split string to doubles
        //cout<<strTemp<<endl;
        std::vector< double > positions= fromString2Array(strTemp);
        //printOutStdVector(positions);
        cv::Point3d currentRobot(positions[0]/1000,positions[1]/1000,positions[2]/1000);
        pts_robot.push_back(currentRobot);
        std::vector<double> currentRobotPose;
        currentRobotPose.push_back(positions[3]);
        currentRobotPose.push_back(positions[4]);
        currentRobotPose.push_back(positions[5]);
        currentRobotPose.push_back(positions[6]);
        pose_robot.push_back(currentRobotPose);

        cout<<"robot position:"<<currentRobot<<endl;
        cout<<"robot pose:";
        printOutStdVector(currentRobotPose);
        cout<<"control point pairs number:"<<pts_marker.size()<<endl;
    }
  }
}
//"[0.53196,1.39524,1.09715,0.955755,-4.0458,-1.292023,-0.000904]"
//"[0.1037,1.167,1.3004,1.27976,-4.7181,-1.54276,-0.0039043]"

int main(int argc, char **argv )
{
  boost::asio::io_service io_service;

    udp::socket s(io_service, udp::endpoint(udp::v4(), 0));

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), UDP_SERVER_IP , PORT);
    udp::resolver::iterator iterator = resolver.resolve(query);

    markerValid=false;
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "chris_calibrator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 1000, ar_pose_marker_callback);
    //ros::Subscriber subP = nh.subscribe("/chris_tracker/currentPoint", 1000, currentPointCallback);
    //ros::Subscriber subPd = nh.subscribe("/chris_tracker/desiredPoint", 1000, desiredPointCallback);
    ros::Publisher pubTask = nh.advertise<std_msgs::Float64MultiArray>("/chris/controlPoints", 1, true);

    //ros::ServiceClient Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
    cout << "Press any key to continue..." << endl;
    getchar();
     // Joint_move_client.call(mv_srv);///////////////////////////////////////////////////

    // cout << "Press any key to continue the servoing loop..." << endl;
    ///  getchar();
    ros::Rate loop_rate(3);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool done=false;
    cout << "UDP client is trying to connect to ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
    while(ros::ok()&&!done)
    {
      ros::spinOnce();

       try {

	  std::cout << "Enter command: ";
	  char request[max_length];
	  std::cin.getline(request, max_length);
	  size_t request_length = strlen(request);
	  string str(request);
	  //std::cout << "you typed:  "+str<<endl;

	  if(str.find("esc") != std::string::npos)
	  {
	    done=true;
	    break;
	  }

	  if(str.find("close connection") != std::string::npos)
	  {
	    std::string closeConnectionCmd = "esc";
	    int myArrayLength=closeConnectionCmd.size();
            char myArray[myArrayLength];//as 1 char space for null is also required
            strcpy(myArray, closeConnectionCmd.c_str());
	    std::cout << "close robot connection command:  "<<closeConnectionCmd<<endl;

	    s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator);
	  }

	  if(str.find("init") != std::string::npos)
	  {
	    std::string Cmd = "init";
	    int myArrayLength=Cmd.size();
            char myArray[myArrayLength];//as 1 char space for null is also required
            strcpy(myArray, Cmd.c_str());
	    std::cout << "initialize command:  "<<Cmd<<endl;

	    s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator);
	  }

    if(str.find("move") != std::string::npos)
    {
      ros::spinOnce();
	    if(!markerValid)
	    {
	      cout<<"marker detection failed, please check marker position!"<<endl;
	      continue;
	    }
	    //get marker position x, y, z
      void add_marker_position();
      //send robot to the position and get robot cartesian coordinates
      std::string Cmd = str;
      int myArrayLength=Cmd.size();
      char myArray[myArrayLength];//as 1 char space for null is not required
      strcpy(myArray, Cmd.c_str());
      std::cout << "command: "+ str <<Cmd<<endl;
      s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator);
      std::cout << "move position command sent:  "+ str <<Cmd<<endl;
      //wait for reply

      resolvRobotPosition();
    }

	  if(str.find("mark") != std::string::npos)//mark as one control point
	  {
	    ros::spinOnce();
	    if(!markerValid)
	    {
	      cout<<"marker detection failed, please check marker position!"<<endl;
	      continue;
	    }
	    //get marker position x, y, z
      void add_marker_position();

	    //get robot position x,y,z
	    std::string getRobotCartPositionCmd = "get_Cart_Position@l@";
	    int myArrayLength=getRobotCartPositionCmd.size();
      char myArray[myArrayLength];//as 1 char space for null is also required
      strcpy(myArray, getRobotCartPositionCmd.c_str());
	    std::cout << "get robot position:  "<<getRobotCartPositionCmd<<endl;

	    s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator);

	    std::cout<<"get robot position cmd sent!"<<endl;
	    //receive position from robot
      resolvRobotPosition();
	  }

	  if(str.find("done") != std::string::npos)//mark as one control point
	  {
	    //publish control points
	    cout<<"control point pairs number:"<<pts_marker.size()<<endl;
// 	    cout<<"markers:"<<pts_marker<<endl;
// 	    cout<<"robots:"<<pts_robot<<endl;

	    cout<<"write to file result.txt"<<endl;
	    ofstream file;
	    file.open("result.txt");

	    size_t cptsNumber=pts_robot.size();
	    for(int m=0;m<cptsNumber;m++)
	      file<<constructCorrsPtsStr(pts_marker[m],pts_robot[m], pose_marker[m], pose_robot[m]);
	    file.close();
	    cout<<"publish correspondent point topic: /chris/control_pts"<<endl;
	    std_msgs::Float64MultiArray corrsPts;

	    for(int m=0;m<cptsNumber;m++)
	    {
	      corrsPts.data.push_back(pts_marker[m].x);
	      corrsPts.data.push_back(pts_marker[m].y);
	      corrsPts.data.push_back(pts_marker[m].z);
	      corrsPts.data.push_back(pose_marker[m][0]);
	      corrsPts.data.push_back(pose_marker[m][1]);
	      corrsPts.data.push_back(pose_marker[m][2]);
	      corrsPts.data.push_back(pose_marker[m][3]);

	      corrsPts.data.push_back(pts_robot[m].x);
	      corrsPts.data.push_back(pts_robot[m].y);
0	      corrsPts.data.push_back(pts_robot[m].z);
	      corrsPts.data.push_back(pose_robot[m][0]);
	      corrsPts.data.push_back(pose_robot[m][1]);
	      corrsPts.data.push_back(pose_robot[m][2]);
	      corrsPts.data.push_back(pose_robot[m][3]);
	    }
	    pubTask.publish(corrsPts);
	    ROS_INFO("control points coordinates published!");
	    pts_robot.clear();
	    pts_marker.clear();
	    pose_marker.clear();
	    pose_robot.clear();
	  }

	  if(str.find("clear") != std::string::npos)//mark as one control point
	  {
	    pts_robot.pop_back();
	    pts_marker.pop_back();
	    pose_marker.pop_back();
	    pose_robot.pop_back();
	    cout<<"this point pair deleted"<<endl;
	    cout<<"control point pairs number:"<<pts_marker.size()<<endl;
	  }

          boost::this_thread::sleep(boost::posix_time::milliseconds(200));
       }
      catch(exception &e) {
          std::cout << "Catch an exception: " << e.what() << std::endl;
       }
    }
}
