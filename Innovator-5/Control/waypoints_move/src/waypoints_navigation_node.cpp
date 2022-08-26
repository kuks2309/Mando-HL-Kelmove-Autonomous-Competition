#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"   
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"


#include <math.h>

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 42
#define WayPoint_X_Tor 0.3//0.4
#define WayPoint_Y_Tor 0.5//0.5

#define V_Region_NO  2
#define V_Speed_Region_NO 3
#define W_Region_NO  1
#define Pass_Region_NO 1
#define Park_Region_NO 1

double pos_x = 0.0;
double pos_y = 0.0;

int vision_steering_angle = 0;
int waypoint_steering_angle = 0;
int car_speed = 0;
int no_waypoints = WayPoints_NO;
int obs = 0;
bool topic_gps_datum_rcv = false;
bool use_utm_absolute_mode = true;
double gps_heading_angle = 0.0;
double waypoint_line_angle = 0.0;
double datum_lat;
double datum_lon;
double datum_yaw;


//init_flag
int init_flag = 0;
int wp_go_id = 0;
int wp_finish_id = 0;

double roll,pitch,yaw;

struct Point 
{ 
	double x; 
	double y; 
	double z;
};

struct WayPoints
{
	double x;
	double y;	
};

struct Rect_Region
{
	double top;
	double bottom;
	double left;
	double right;	
};

geometry_msgs::Pose2D my_pose;
geometry_msgs::Pose2D my_target_pose_goal;
geometry_msgs::Pose2D my_target_pose_goal_prev;
geometry_msgs::Pose2D my_pose_utm_waypoint;    // coordinate,  current waypoint to goal waypoint
geometry_msgs::Pose2D initial_utm_pose;


struct Rect_Region Vision_Region[V_Region_NO];
struct Rect_Region Vision_Speed_Region[V_Region_NO];
struct Rect_Region WayPoint_Region[W_Region_NO];
struct WayPoints my_waypoints_list[WayPoints_NO];
struct Rect_Region Passing_Region[Pass_Region_NO];
struct Rect_Region Parking_Region[Park_Region_NO];

//GPS의경우 UTM 좌표를 따라서 XY가 다름

void gps_utm_poseCallback(const geometry_msgs::Pose2D& msg)
{
	my_pose.x     =  msg.x;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
	my_pose.y     =  msg.y;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
	my_pose.theta =  msg.theta;
	
//if(msg.theta <=0) my_pose.theta =  msg.theta + 2*M_PI;   
}

void gps_datum_Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	
	printf("GPS Datum RCV!\n");
	topic_gps_datum_rcv = true;
    datum_lat = msg->x;               
    datum_lon = msg->y;
    datum_yaw = msg->z;  
	
}
void waypointstartIDCallback(const std_msgs::Int16& msg)
{	
	wp_go_id  = msg.data; 
}

void waypointfinishIDCallback(const std_msgs::Int16& msg)
{	
	wp_finish_id  = msg.data; 
}

void GPSHeadingAngleCallback(const std_msgs::Float32& msg)
{
	gps_heading_angle = msg.data;   // radian 으로 받을 것
}

void odomCallback(const nav_msgs::Odometry& msg)
{
	my_pose.x = (double)msg.pose.pose.position.x;
	my_pose.y = (double)msg.pose.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.pose.orientation.x,        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,        msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);     
 
    m.getRPY(roll, pitch, yaw);
    my_pose.theta = yaw;		
}

void init_waypoint_region(void)
{
	FILE *fp;
	
	fp= fopen("//home//amap//waypoints//waypoint_region.txt","r");
	
	if(fp == NULL)
	{
		ROS_INFO("Waypoint_region does not exit!");
		WayPoint_Region[0].top    =  7.6;
	    WayPoint_Region[0].bottom =  7.2;
	    WayPoint_Region[0].left   =  -4.9; 
	    WayPoint_Region[0].right  =  -5.1;
   }
   else
   {
	   fscanf(fp,"%lf %lf %lf %lf",&WayPoint_Region[0].top, &WayPoint_Region[0].bottom, &WayPoint_Region[0].left, &WayPoint_Region[0].right);
	   fclose(fp);
   }
}
 
void init_waypoint(void)
{
	FILE *fp;
	
	fp= fopen("//home//amap//amap//waypoints//waypoints_data.txt","r");
	
	//fp = NULL;
	
	if(fp == NULL)
	{
		ROS_INFO("Waypoints_data does not exit!");
		
	    my_waypoints_list[0].x = 1;   
        my_waypoints_list[0].y = 1;
	
	    my_waypoints_list[1].x = 3;   
        my_waypoints_list[1].y = 3;
  
        my_waypoints_list[2].x = 2;   
        my_waypoints_list[2].y = 6;  		
 
        my_waypoints_list[3].x = 3;   
        my_waypoints_list[3].y = 10; 
        
        no_waypoints = 4;
        wp_finish_id = no_waypoints;
   }
   else
   {
	    fscanf(fp,"%d",&no_waypoints);
	    
	    wp_finish_id = no_waypoints;
	  
	    for(int i=0; i<no_waypoints; i++)
	    {
			fscanf(fp,"%lf %lf",&my_waypoints_list[i].x, &my_waypoints_list[i].y);
	    }
	  
	    ROS_INFO("WayPoints Number %d",WayPoints_NO);
	    for(int i=0; i<no_waypoints; i++)
	    {
			ROS_INFO("WayPoints-%d : [%.2lf]%.2lf]",i,my_waypoints_list[i].x,my_waypoints_list[i].y);
	    }
	    fclose(fp);
   }
}

void WaySteerControlCallback(const std_msgs::Int16& angle)
{
  waypoint_steering_angle = (int)(angle.data) ;
 
  if(waypoint_steering_angle >= MAX_R_STEER)  waypoint_steering_angle = MAX_R_STEER;
  if(waypoint_steering_angle <= MAX_L_STEER)  waypoint_steering_angle = MAX_L_STEER;  
}


void obs_detect_Callback(const std_msgs::Int8& msg)
{
	obs = msg.data;
	printf("obs : %d\n", obs);
}

void waypoint_tf(void)
{
	double x,y;
	double tf_waypoint_x,tf_waypoint_y; 
	x=y=0;
	
	tf_waypoint_x =  my_pose.x - my_target_pose_goal_prev.x;
	tf_waypoint_y =  my_pose.y - my_target_pose_goal_prev.y;  
		
	my_pose_utm_waypoint.x =  tf_waypoint_x * cos(-waypoint_line_angle) -  tf_waypoint_y * sin(-waypoint_line_angle);   // rotation_matrix
	my_pose_utm_waypoint.y =  tf_waypoint_x * sin(-waypoint_line_angle) +  tf_waypoint_y * cos(-waypoint_line_angle);   	; 
		
	//printf("relative my pose at waypoint tf :%6.3lf , %6.3lf \n", my_pose_utm_waypoint.x, my_pose_utm_waypoint.y);
		
}
void base_link_tf_utm(void)
{
	
	double waypoint_pos_base_link_x     = 0.0;
	double waypoint_pos_base_link_y     = 0.0; 
	double waypoint_pos_base_link_theta = 0.0; 
	double tf_base_map_x,tf_base_map_y; 
	double waypoint_angle, waypoint_distance;	 
	
	tf_base_map_x = -my_pose.x;   //상대좌표로 변환  no translation
	tf_base_map_y = -my_pose.y;
	
	tf_base_map_x += my_target_pose_goal.x;   //상대좌표로 변환  no translation
	tf_base_map_y += my_target_pose_goal.y;     
    
    // my_pose.theta = M_PI_2;     
	waypoint_pos_base_link_x = tf_base_map_x * cos(my_pose.theta)  + tf_base_map_y * sin(my_pose.theta);   // rotation_matrix
	waypoint_pos_base_link_y = -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta);   	
	
	waypoint_angle = atan2(waypoint_pos_base_link_y ,waypoint_pos_base_link_x);	
	waypoint_distance = sqrt(waypoint_pos_base_link_x*waypoint_pos_base_link_x  + waypoint_pos_base_link_y*waypoint_pos_base_link_y);
    
    ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf ", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta)); 
    ROS_INFO(" b_x : %6.3lf  b_y : %6.3lf",waypoint_pos_base_link_x,waypoint_pos_base_link_y);  
	    
	
}


void check_inside_waypoint(int waypoint_id)
{
	double  waypt_line_angle = 0.0;
	
	//printf("%d %d \n", wp_go_id, no_waypoints);
	    if(waypoint_id != 0)
	    { 
	        my_target_pose_goal_prev.x = my_waypoints_list[waypoint_id-1].x ;//- initial_utm_pose.x;
	        my_target_pose_goal_prev.y = my_waypoints_list[waypoint_id-1].y ;//- initial_utm_pose.x;
	        
	        double delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
	        double delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
	        waypt_line_angle = atan2(delta_y, delta_x);
	        
	        printf("1: tf angle %lf\n", RAD2DEG( waypt_line_angle)); 
	       // printf("%6.3lf , %6.3lf \n",my_target_pose_goal_prev.x, my_target_pose_goal_prev.y);
	       // printf("%6.3lf , %6.3lf \n",my_target_pose_goal_prev.x, my_target_pose_goal_prev.y);
		}

}



int main(int argc, char **argv)
{
  //FILE* error_data = fopen("/home/chan/바탕화면/Clear_Robot_catkin_ws/Steering/include/data/error_data_1.txt","a");
  char buf[2];
  ros::init(argc, argv, "cleanbot_race_waypoints_manager_utm");

  ros::NodeHandle n;
  
  std_msgs::Int16 s_angle;
  std_msgs::Int16 c_speed;
  std_msgs::Int16 ros_waypoint_id;
  std_msgs::String slam_reset;
  
  geometry_msgs::Pose2D gps_init_pose2d_data;

  slam_reset.data = "reset";
  
  datum_lat = datum_lon =  datum_yaw = 0.0; 
   
  ros::Subscriber sub1 = n.subscribe("/Car_Control_cmd/W_SteerAngle_Int16",10, &WaySteerControlCallback);
  ros::Subscriber sub2 = n.subscribe("/gps/utm_pos1",10, &gps_utm_poseCallback);
    
  ros::param::get("~use_utm_absolute_mode", use_utm_absolute_mode);        //2개의 GPS를 사용할 경우 yaw값은 2개의 GPS로 부터 계산함   
  
  
  ros::Subscriber sub_gps_datum = n.subscribe("/gps/datum",1,&gps_datum_Callback);  // front gps      
  ros::Subscriber sub3 = n.subscribe("/start_waypoint_id_no",1, &waypointstartIDCallback);
  ros::Subscriber sub4 = n.subscribe("/finish_waypoint_id_no",1, &waypointfinishIDCallback);
  ros::Subscriber sub5 = n.subscribe("/gps_heading_angle",1,&GPSHeadingAngleCallback);
 
 
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_Int16", 10);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 10);
  ros::Publisher target_id_pub    = n.advertise<std_msgs::Int16>("target_id",2);
  ros::Publisher target_pos_pub   = n.advertise<geometry_msgs::Pose2D>("/pose_goal", 10);
  
  ros::Publisher waypoint_guide_line_pub = n.advertise<nav_msgs::Path>("/waypoint_guide_line",1, true);

   
  ros::Rate loop_rate(5);  // 10 

  //GSP_init_datum.data[0] = 10.;
  //GSP_init_datum.data[1] = 11.;

   
  long count = 0;
  int mission_flag[WayPoints_NO] = {0,};
  double pos_error_x = 0.0;
  double pos_error_y = 0.0;

  double waypoint_distance = 0.0;
  double waypoint_gap_distance = 0.0;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  
  geometry_msgs::Pose2D pose_goal;  
    
  
  init_waypoint(); 
  
  initial_utm_pose.x = datum_lat;  //gps datum 처리 
  initial_utm_pose.y = datum_lon;  //gps datum 처리

  
  int vision_id = -1;
  int vision_speed_id = -1;
  int waypoint_id = 0;
 
  double start_x = 0.;  
  double start_y = 0.;
  //init_waypoint_region();
  
  double delta_x, delta_y ;
  delta_x = delta_y = 0.0;
  
  double base_line_a, base_line_b; // waypoint line between start and target point
   
  pose_goal.x = my_waypoints_list[wp_go_id].x;
  pose_goal.y = my_waypoints_list[wp_go_id].y;
  pose_goal.theta = DEG2RAD(0);
  target_pos_pub.publish(pose_goal);   
  
  if(count == 0)
  {
	start_x = my_pose.x;   start_y = my_pose.y;
	start_x = 0;   start_y = 0;
  }  
 
  
  if(use_utm_absolute_mode == true)
     {
	     ROS_INFO(" ");
	     ROS_INFO(" ");
	     ROS_INFO("nutm absolute mode");
	     ROS_INFO(" ");
	     ROS_INFO(" ");
	     ros::Duration(3.0).sleep() ;          
     }
    else
    {
	    
	     ROS_INFO(" ");
	     ROS_INFO(" ");
	     ROS_INFO("\n\n\n\nutm relative mode\n\n\n\n");  
	     ROS_INFO(" ");
	     ROS_INFO(" ");
	     printf("topic_gps_datum_mode  : %d\n", topic_gps_datum_rcv );
	     /*
	     while(topic_gps_datum_rcv == false)
	     {
			 ROS_WARN_STREAM("utm relative mode now: waiting topic GPS Datum");
			 
		 }
	     */
	     ros::Duration(3.0).sleep() ;
    }
   
   
  
 
  while (ros::ok())
  {	
    
    gps_init_pose2d_data.x     = my_waypoints_list[0].x;
    gps_init_pose2d_data.y     = my_waypoints_list[0].y;
    gps_init_pose2d_data.theta =  0;
        
   
    
    if(waypoint_id!= -1)
    {	
		
	    //check_inside_waypoint(1);
		//ROS_INFO("WayPoint Number : %d", no_waypoints);
	    //ROS_INFO("WayPoint Region : %2d",waypoint_id);
	    
	    my_target_pose_goal.x = my_waypoints_list[wp_go_id].x ;//- initial_utm_pose.x;
	    my_target_pose_goal.y = my_waypoints_list[wp_go_id].y ;//- initial_utm_pose.y;
	    
	    
	  //printf("goal_pose : %6.3lf %6.3lf \n", my_target_pose_goal.x , my_target_pose_goal.y);
	  //printf("%d %d \n", wp_go_id, no_waypoints);
	    
		if(wp_go_id == 0) 
		{
			delta_x = my_waypoints_list[1].x - my_waypoints_list[0].x;
	        delta_y = my_waypoints_list[1].y - my_waypoints_list[0].y;
	        
			
			my_target_pose_goal_prev.x = my_waypoints_list[0].x - delta_x;
	        my_target_pose_goal_prev.y = my_waypoints_list[0].y - delta_y;
	        
	        delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
	        delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
	        waypoint_line_angle = atan2(delta_y, delta_x);
	     
	       // printf("start_pose : %6.3lf %6.3lf \n", start_x, start_y);   
	       //printf("2: angle %lf\n", RAD2DEG(waypoint_line_angle)); 			
		}
		else
	    { 		  
	        my_target_pose_goal_prev.x = my_waypoints_list[wp_go_id-1].x;// - initial_utm_pose.x;
	        my_target_pose_goal_prev.y = my_waypoints_list[wp_go_id-1].y;// - initial_utm_pose.y;
	        
	        delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
	        delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
	        waypoint_line_angle = atan2(delta_y, delta_x);
	        
	       // printf("1: angle %lf\n", RAD2DEG(waypoint_line_angle)); 	        
		}
				
		waypoint_distance = sqrt(delta_x*delta_x + delta_y*delta_y);		
	
		waypoint_gap_distance = sqrt(delta_x*delta_x + delta_y*delta_y) - WayPoint_X_Tor;
		//printf("gap : %6.3lf  \n",waypoint_gap_distance);
				
		printf("1: angle %lf\n", RAD2DEG(waypoint_line_angle)); 	
		
			    
	    pos_error_x = abs(my_pose.x - my_waypoints_list[wp_go_id].x);
	    pos_error_y = abs(my_pose.y - my_waypoints_list[wp_go_id].y); 
	   	     
	    pose_goal.x =  my_waypoints_list[wp_go_id].x ;;//- initial_utm_pose.y;    // umt coordinate
	    pose_goal.y =  my_waypoints_list[wp_go_id].y;; // + initial_utm_pose.x);   // umt coordinate 
	    
	    
	    pose_goal.theta = DEG2RAD(0);
	    ROS_INFO("[%3d]WayPoint goal X : %6.3lf  goal Y : %6.3lf ",wp_go_id, my_target_pose_goal.x, my_target_pose_goal.y);  
	    
	    
	    waypoint_tf();
	    base_link_tf_utm();
	    
	    
	    target_pos_pub.publish(pose_goal);
	    ros_waypoint_id.data  = wp_go_id;
	    
	    

	    //printf("d_x %6.3lf d_y %6.3lf \n", delta_x,delta_y);
	    //printf("waypoint_distance  %6.3lf \n", waypoint_distance);
	    /*
	    if( (count>=0) && ( my_pose_utm_waypoint.x  >=(waypoint_distance -  WayPoint_X_Tor) ) && ( my_pose_utm_waypoint.x <=(waypoint_distance +  WayPoint_X_Tor) )  )
        {           
	       printf("----------------------------\n"); 
           printf("Arrvied at My WayPoint[%3d] !\n",wp_go_id); 
           printf("----------------------------\n"); 
           
           ros::Duration(4.0).sleep() ;      
	       count = -3;
	       wp_go_id++;
	     }
	   */
	     if( (count>=0) && (pos_error_x <= WayPoint_X_Tor) && (pos_error_y <= WayPoint_Y_Tor )  )
        {           
	       printf("----------------------------\n"); 
           printf("Arrvied at My WayPoint[%3d] !\n",wp_go_id); 
           printf("----------------------------\n"); 
           
           ros::Duration(4.0).sleep() ;      
	       count = -3;
	       wp_go_id++;
	     }
	      
	    
	    s_angle.data = waypoint_steering_angle;
	    
	    if(obs == 1)
	    {
			c_speed.data =0;
		}
		else
		{
			
		}
		c_speed.data = 10;
		
	    if(wp_go_id >= wp_finish_id) 
	    {
			 c_speed.data = 0;
			 wp_go_id = wp_finish_id;
			 ROS_INFO("WP Mission Completed");	
				 }
	}	
	
	// publish topics	
	target_id_pub.publish(ros_waypoint_id);
	ROS_INFO("steering_angle : %d Speed : %d \n",s_angle.data ,c_speed.data);
	if(count>=2)
	{
	    car_control_pub1.publish(s_angle);
	    car_control_pub2.publish(c_speed);
	}	
	 
	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

