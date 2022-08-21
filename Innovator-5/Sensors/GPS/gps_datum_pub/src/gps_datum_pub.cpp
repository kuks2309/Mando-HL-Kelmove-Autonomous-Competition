#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "geometry_msgs/Vector3.h"
#include <XmlRpcException.h>

// Datum parameter - required
double datum_lat;
double datum_lon;
double datum_yaw;
  
  
int main(int argc, char **argv)
{
    

	ros::init(argc, argv, "GPS_Datum_Publisher");

	ros::NodeHandle n;
    ros::NodeHandle nh_priv("~");
    
    ros::Publisher pub = n.advertise<geometry_msgs::Vector3>("array", 10);
    
    if(!nh_priv.hasParam("datum") )  
    {
      ROS_FATAL("private <datum> parameter is not supplied in "  "geonav_transform configuration");
      exit(1);
    }

    XmlRpc::XmlRpcValue datum_config;    
    nh_priv.getParam("datum", datum_config);
    
    ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(datum_config.size() >= 3);
      
    if (datum_config.size() > 3)
    {
	    ROS_WARN_STREAM("Deprecated datum parameter configuration detected. "
			"Only the first three parameters "
			"(latitude, longitude, yaw) will be used. frame_ids "
			"will be derived from odometry and navsat inputs.");
    }
      // Parse the data spec.
    std::ostringstream ostr;
    ostr << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
    std::istringstream istr(ostr.str());
    istr >> datum_lat >> datum_lon >> datum_yaw;
    
    ROS_INFO("GPS Datum [%12.9lf %12.9lf %12.9lf]", datum_lat, datum_lon, datum_yaw);
    
    ros::Rate loop_rate(10);  // 10
	while (ros::ok())
	{
		geometry_msgs::Vector3 GPS_Datum;
		//Clear array
		
		GPS_Datum.x = datum_lat;
		GPS_Datum.y = datum_lon;
		GPS_Datum.z = datum_yaw;
		
		//for loop, pushing data in the size of the array		
		
		//Publish array
		pub.publish(GPS_Datum);
		//Let the world know
		ROS_INFO("GPS Datum [%12.9lf %12.9lf %12.9lf]", datum_lat, datum_lon, datum_yaw);
		//Do this.
		loop_rate.sleep();
        ros::spinOnce();
	}

}
