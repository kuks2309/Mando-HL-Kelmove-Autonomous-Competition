//2022.08.21 gps/datum
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 50

int no_waypoints;

double datum_lat;
double datum_lon;
double datum_yaw;
int use_gps_init_datum;


struct Point3D
{
  float x;
  float y;
  float z;
};

struct WayPoints
{
	double x;
	double y;	
};


struct WayPoints my_waypoints_list[WayPoints_NO];


void gps_datum_Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  //printf("datum test\n");	
  datum_lat = msg->x;               
  datum_lon = msg->y;
  datum_yaw = msg->z;
	
}


void init_waypoint(void)
{
	FILE *fp;
	
	fp= fopen("//home//amap//amap//waypoints//waypoints_data.txt","r");
	
	if(fp == NULL)
	{
		ROS_INFO("Waypoints_data does not exit!");
		
	    my_waypoints_list[0].x = 1;   
        my_waypoints_list[0].y = 2;
	
	    my_waypoints_list[1].x = 1;   
        my_waypoints_list[1].y = 4;
  
        my_waypoints_list[2].x = 2;   
        my_waypoints_list[2].y = 6;  		
 
        my_waypoints_list[3].x = 3;   
        my_waypoints_list[3].y = 10; 
        
        no_waypoints = 4;
   }
   else
   {
	    fscanf(fp,"%d",&no_waypoints);
	  
	    for(int i=0; i<no_waypoints; i++)
	    {
			fscanf(fp,"%lf %lf",&my_waypoints_list[i].x, &my_waypoints_list[i].y);
	    }
	  
	    ROS_INFO("WayPoints Number %d",no_waypoints);
	    for(int i=0; i<no_waypoints; i++)
	    {
			ROS_INFO("WayPoints-%d : [%.2lf]%.2lf]",i,my_waypoints_list[i].x,my_waypoints_list[i].y);
	    }
	    fclose(fp);
   }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_marker_display");
  ros::NodeHandle n;
    
  
  datum_lat = datum_lon = 0.0; 
   
  std::vector<double> gps_init_datum; 
  ros::param::get("/gps_init_datum",  gps_init_datum); //gps datum 수신 향후 처리
  ros::param::get("/use_gps_init_datum",  use_gps_init_datum); //gps datum 수신 향후 처리
  
  
  datum_lat = 0;               
  datum_lon = 0;
  datum_yaw = 0;
  if(use_gps_init_datum == 1)
  {
     datum_lat = gps_init_datum[0];               
     datum_lon = gps_init_datum[1];
     datum_yaw = gps_init_datum[2];
  }
  
  ros::Subscriber sub_navsat_fix1 = n.subscribe("/gps/datum",1,&gps_datum_Callback);  // front gps 
  
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker/node", 1);
  
  Point3D p;
  std::vector<Point3D> vec_point;
  
  init_waypoint();
  // 초기화 
  for(int i=0; i<no_waypoints; i++)
  {
   
    
    if(use_gps_init_datum == 1) 
    {
	   p.x = my_waypoints_list[i].x - datum_lat;
       p.y = my_waypoints_list[i].y - datum_lon;
    	
	}
	
	else
	{
		 p.x = my_waypoints_list[i].x - my_waypoints_list[0].x;
         p.y = my_waypoints_list[i].y - my_waypoints_list[0].y;;
	}
    p.z = 0;
    vec_point.push_back(p);
  }
  visualization_msgs::MarkerArray node_arr;
  
  for (size_t i = 0; i < vec_point.size(); i++)
  {
        Point3D o_node = vec_point[i];

        visualization_msgs::Marker node;
        node.header.frame_id = "/map"; // map frame 기준
        node.header.stamp = ros::Time::now();
        node.type = visualization_msgs::Marker::SPHERE;
        node.id = i;
        node.action = visualization_msgs::Marker::ADD;
        node.pose.orientation.w = 1.0;
        node.pose.position.x =  o_node.x; //노드의 x 좌표
        node.pose.position.y =  o_node.y; //노드의 y 좌표
        // Points are green
        node.color.g = 0.5;
        node.color.a = 1.0;
        node.scale.x = 0.5;                       // marker size
        node.scale.y = 0.5;                       // marker size
        node_arr.markers.push_back(node);
  }
  
  ros::Rate loop_rate(1);  // 10
  
  while (ros::ok())
  {
	marker_pub .publish(node_arr);  
	loop_rate.sleep();
    ros::spinOnce();

  }
  return 0;
}
