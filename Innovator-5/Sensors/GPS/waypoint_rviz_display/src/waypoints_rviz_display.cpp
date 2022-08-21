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
double datum_utm_east;
double datum_utm_north;

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

void wgs2utm(double lat, double lon, int zone , double& east, double& north){
    double lat_rad = lat * M_PI/180;
    double lon_rad = lon * M_PI/180;

    double phi = lat_rad;
    double lambda = lon_rad;
    double lambda0 = (zone * 6 -183) * M_PI/180;
    double sm_a = 6378137;
    double sm_b = 6356752.31;

    double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
    double nu2 = ep2*pow(cos(phi), 2.0);
    double N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
    double l = lambda - lambda0;
    double t = tan(phi);
    double t2 = t * t;

    double l3coef = 1 - t2 + nu2;
    double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    east = N * cos(phi) * l + 
        (N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + 
        (N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + 
        (N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    double n = (sm_a - sm_b) / (sm_a + sm_b);
    double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n,4.0) / 64.0));
    double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
    double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n,4.0) / 32.0);
    double delta = (-35.0 * pow(n,3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
    double epsilon = (315.0 * pow(n, 4.0) / 512.0);

    double ArcLengthMeridian = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0  * phi)) + (epsilon * sin(8.0 * phi)));

    north = ArcLengthMeridian + 
            (t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + 
            (t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + 
            (t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + 
            (t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
}


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
  
  //printf("use_gps_init_datum : %d \n\n",use_gps_init_datum);
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
  double zone;
  init_waypoint();
  
  for(int i=0; i<no_waypoints; i++)
  {      
	if(use_gps_init_datum == 1) 
	{
		//printf("use param datum initialization\n\n");
		p.x = my_waypoints_list[i].x - my_waypoints_list[0].x;
        p.y = my_waypoints_list[i].y - my_waypoints_list[0].y;;
	}
    p.z = 0.0;
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
        node.scale.z = 0.02;                       // marker size
        node_arr.markers.push_back(node);
  } 
  
  ros::Rate loop_rate(1);  // 10
  
  while (ros::ok())
  {
	 // 초기화 
    if(use_gps_init_datum == 0) 
    {
	   vec_point.clear();
	     
       if(datum_lon < 0)
       {
          zone = (datum_lon + 180) / 6 + 1;
       }
       else
       {
         zone = datum_lon / 6 + 31;
       }
	  
	   wgs2utm( datum_lat,datum_lon, zone, datum_utm_east, datum_utm_north);
	   
	   datum_utm_east = datum_utm_east * 0.9996 + 500000;
       datum_utm_north = datum_utm_north * 0.9996;
	   
	  // printf("use topic datum initialization %8.4lf %8.4lf %8.4lf %8.4lf \n\n", datum_lat, datum_lon, datum_utm_east, datum_utm_north);  
	    
       for(int i=0; i<no_waypoints; i++)
       {
	      p.x = my_waypoints_list[i].x - datum_utm_east;
          p.y = my_waypoints_list[i].y - datum_utm_north;
          p.z = 0.0;
          printf("%d %lf %lf %lf %lf \n",i,my_waypoints_list[i].x, my_waypoints_list[i].y, datum_utm_east, datum_utm_north);
          node_arr.markers[i].pose.position.x = p.x;
          node_arr.markers[i].pose.position.y = p.y;
	   }
    	
	}	  
	  
	marker_pub .publish(node_arr);  
	loop_rate.sleep();
    ros::spinOnce();

  }
  return 0;
}
