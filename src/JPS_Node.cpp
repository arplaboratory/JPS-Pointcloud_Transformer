  #include<iostream>
#include<stdio.h>
#include<cstdlib>
#include<cmath>
#include<vector>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include<Eigen/Geometry>
#include<Eigen/Dense> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<JPS/path_planning/JumpPointSearch.h>
#include<visualization_msgs/MarkerArray.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include<topic_tools/shape_shifter.h>
#include "JPS/jps_service_message.h"
#include "JPS/jps_waypoint_service.h"
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
static int counter = 0;
class JPSNodelet
{
public:
Eigen::Matrix4d Trans_Matrix;
Eigen::Matrix4d inv_Trans_Matrix;
int x_dest;
int y_dest;
int z_dest;
int map_size;
int offset;
int offset_z;
double cell_size;
int max_iterations;
int pad_size;
double threshold;
int waypoint_num;
bool is_dest_set;
bool is_waypoint_set;
nav_msgs::Path waypoints;
boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > > odom_msg;
boost::shared_ptr<const topic_tools::ShapeShifter> map_msg;
void pointcloud_subscriberCallback(const topic_tools::ShapeShifter::ConstPtr& msg);
void pose_callback(const nav_msgs::Odometry::ConstPtr& odom);
void find_free_node(int ***ogm,int &x, int &y, int &z, int max_scope);
void JPS_Publisher(int x_dest, int y_dest, int z_dest);
void Visualiser(vector <struct path_planning::JumpPointSearch::Node> rpath);
bool service_callback(JPS::jps_service_message::Request &req, JPS::jps_service_message::Response &res);
bool waypoint_service_callback(JPS::jps_waypoint_service::Request &req, JPS::jps_waypoint_service::Response &res);
path_planning::JumpPointSearch jps;
ros::NodeHandle n;
ros::Publisher marker_pub;
ros::Publisher waypoint_pub;
ros::Subscriber pc_sub;
ros::Subscriber pose_sub;
ros::ServiceServer service;
ros::ServiceServer waypoint_service;
nav_msgs::Path goal_waypoints;

JPSNodelet()
{//"/race2/odom_transform_nodelet/odomBinB0_from_transform"
 //"/race5/odom_transform_nodelet/odomBinB0_from_transform"
 odom_msg = NULL;
 map_msg = NULL;
 is_dest_set = false;
 is_waypoint_set = false;
 n = ros::NodeHandle("/quadrotor/jps_server");
 marker_pub = n.advertise<visualization_msgs::Marker>("/JPS_Path", 1);
 waypoint_pub = n.advertise<nav_msgs::Path>("/quadrotor/waypoints",1);
 pose_sub = n.subscribe("/quadrotor/odom",1,&JPSNodelet::pose_callback,this);
 pc_sub = n.subscribe("/voxblox_node/occupied_nodes",1,&JPSNodelet::pointcloud_subscriberCallback,this);
 service = n.advertiseService("/JPS_Server_single_waypoint",&JPSNodelet::service_callback,this);
 waypoint_service = n.advertiseService("/JPS_Server_multiple_waypoints",&JPSNodelet::waypoint_service_callback,this);
 n.param("map_size",map_size,10);
 n.param("cell_size",cell_size,0.4);
 n.param("max_iterations",max_iterations,500);
 n.param("pad_size",pad_size,1);
 n.param("threshold",threshold,1.0);
 n.param("waypoint_num",waypoint_num,2);
 Trans_Matrix.setZero();
 inv_Trans_Matrix.setZero();
 offset_z = offset = int(map_size/2);
 waypoints.header.frame_id = "world";
 jps.set_mapsize(map_size);
 jps.set_max_iter(max_iterations);
}

};
void JPSNodelet::pose_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
 odom_msg = odom;
 Eigen::Quaterniond q(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
 q.normalize();
 Eigen::Matrix3d Rot_matrix;
 Rot_matrix = q.toRotationMatrix();
 offset_z = int(round(odom->pose.pose.position.z/cell_size));
 if (offset_z > offset)
 {offset_z = offset;}
 Trans_Matrix << Rot_matrix(0,0),Rot_matrix(0,1),Rot_matrix(0,2),odom->pose.pose.position.x,
 		  Rot_matrix(1,0),Rot_matrix(1,1),Rot_matrix(1,2),odom->pose.pose.position.y,
 		  Rot_matrix(2,0),Rot_matrix(2,1),Rot_matrix(2,2),odom->pose.pose.position.z,
 		  0.0, 0.0, 0.0, 1.0;
 inv_Trans_Matrix = Trans_Matrix.inverse();
}

void JPSNodelet::find_free_node(int ***ogm,int &x, int &y, int &z, int max_scope)
{
 if (ogm[x][y][z] == 1)
 {return;
 }
 int scope = 1;
 while (scope < max_scope)
 {
	 for (int i = -scope; i <= scope; i = i + 2*scope)
	 {
		 for (int j = -scope+1; j <= scope-1; j++)
		 {
			 for (int k = -scope+1; k <= scope-1; k++)
			 {
			  if (((x + i)>=0) && ((x + i)<=map_size-1) && ((y + j)>=0) && ((y + j)<=map_size-1) && ((z + k)>=0) && ((z + k)<=map_size-1))
			  {
			  	if (ogm[x+i][y+j][z+k] == 1)
			  	{
			  	 x += i;
			  	 y += j;
			  	 z += k;
			  	 return;
			  	 
			  	}
        }
       }
     }
   }
   for (int k = -scope; k <= scope; k++)
	 {
		 for (int j = -scope; j <= scope; j++)
		 {
       if (((z-scope)>=0) && ((y + j)>=0) && ((y + j)<=map_size-1) && ((x + k)>=0) && ((x + k)<=map_size-1))
       {
        if(ogm[x+k][y+j][z-scope] == 1)
        {
           x += k;
			  	 y += j;
			  	 z -= scope;
			  	 return;
        }
       }
       else if (((z + scope)<=map_size-1) && ((y + j)>=0) && ((y + j)<=map_size-1) && ((x + k)>=0) && ((x + k)<=map_size-1))
       {
        if(ogm[x+k][y+j][z+scope] == 1)
        {
           x += k;
			  	 y += j;
			  	 z += scope;
			  	 return;
        }
       }
       if (((x + k)>=0) && ((x + k)<=map_size-1) && ((y + scope)<=map_size-1) && ((z + j)>=0) && ((z + j)<=map_size-1))
       {
         if(ogm[x+k][y+scope][z+j] == 1)
        {
           x += k;
			  	 y += scope;
			  	 z += j;
			  	 return;
        }
       }
       if (((x + k)>=0) && ((x + k)<=map_size-1) && ((y - scope)>=0) && ((z + j)>=0) && ((z + j)<=map_size-1))
       {
        if(ogm[x+k][y-scope][z+j] == 1)
        {
           x += k;
			  	 y -= scope;
			  	 z += j;
			  	 return;
        }
		   }
		 }
	 }
	 scope += 1;
 }
 ROS_INFO("Unable to find a nearby empty node!!!\n");
 is_dest_set = false;
 return;
}

void JPSNodelet::Visualiser(vector <struct path_planning::JumpPointSearch::Node> rpath)
{
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "world";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "Jump_Point_Search";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;


    points.id = 0;
    line_strip.id = 1;
   

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    line_strip.scale.x = 0.1;

    points.color.g = 1.0;
    points.color.a = 1.0;
    

    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    
    
    waypoints.poses.clear();
    waypoints.header.stamp = ros::Time::now();
    int k, i;
  for(k = rpath.size()-2,i = 0;k>=0; i++, k--)
	 {geometry_msgs::Point p;
		Eigen::Vector4d vec_Drone_Frame((rpath[k].current_node_x - offset)*cell_size, (rpath[k].current_node_y - offset)*cell_size,(rpath[k].current_node_z - offset_z)*cell_size,1.0);
    Eigen::Vector4d vec_World_Frame = Trans_Matrix*vec_Drone_Frame; 
		p.x = vec_World_Frame(0);
		p.y = vec_World_Frame(1);
		p.z = vec_World_Frame(2);
    if(i<waypoint_num)
    {
    geometry_msgs::PoseStamped w;
    w.header.frame_id = "world";
		w.header.stamp = ros::Time::now();
    w.pose.position.x = vec_World_Frame(0);
		w.pose.position.y = vec_World_Frame(1);
		w.pose.position.z = vec_World_Frame(2); 
		w.pose.orientation.x = 0.0;
		w.pose.orientation.y = 0.0;
		w.pose.orientation.z = 0.0;
		w.pose.orientation.w = 1.0;
    waypoints.poses.push_back(w);
    }
		points.points.push_back(p);
		line_strip.points.push_back(p);
   }
         marker_pub.publish(points);
         marker_pub.publish(line_strip);
         waypoint_pub.publish(waypoints);

         ROS_INFO("Publishing..\n");
}


void JPSNodelet::JPS_Publisher(int x_dest, int y_dest, int z_dest)
{
 Eigen::Vector4d dest(x_dest,y_dest,z_dest,1);
 Eigen::Vector4d dest_rel = inv_Trans_Matrix*dest;
 int dest_rel_x = int(round(dest_rel(0)));
 int dest_rel_y = int(round(dest_rel(1)));
 int dest_rel_z = int(round(dest_rel(2)));
 dest_rel_x = dest_rel_x/cell_size + offset;
 dest_rel_y = dest_rel_y/cell_size + offset;
 dest_rel_z = dest_rel_z/cell_size + offset_z;
 int start_x = offset;
 int start_y = offset;
 int start_z = offset_z;
 if (dest_rel_x<0)
 {dest_rel_x = 0;}
 else if (dest_rel_x>(map_size-1))
 {dest_rel_x = (map_size-1);}
 
 if (dest_rel_y<0)
 {dest_rel_y = 0;}
 else if (dest_rel_y>(map_size-1))
 {dest_rel_y = (map_size-1);}
 
 if (dest_rel_z<0)
 {dest_rel_z = 0;}
 else if (dest_rel_z>(map_size-1))
 {dest_rel_z = (map_size-1);}
 if (jps.ogm[dest_rel_x][dest_rel_y][dest_rel_z] == 0)
 {
  //Eigen::Vector4d d((dest_rel_x-offset)*cell_size, (dest_rel_y-offset)*cell_size,(dest_rel_z-offset_z)*cell_size,1.0);
  //Ei  ROS_INFO("Acquiring Path.....");gen::Vector4d de = Trans_Matrix*d;
  //ROS_INFO("Destination Node is an occupied node!! Finding nearby free node\n");
  //ROS_INFO("Before: %f, %f, %f \n",de(0), de(1),de(2));
  find_free_node(jps.ogm,dest_rel_x,dest_rel_y,dest_rel_z,map_size);
  //Eigen::Vector4d da((dest_rel_x-offset)*cell_size, (dest_rel_y-offset)*cell_size,(dest_rel_z-offset_z)*cell_size,1.0);
  //Eigen::Vector4d des = Trans_Matrix*da;
  //ROS_INFO("After: %f, %f, %f \n",des(0), des(1),des(2));

 }
 if (jps.ogm[start_x][start_y][start_z] == 0)
 {
  find_free_node(jps.ogm,start_x,start_y,start_z,pad_size+3);
  //ROS_INFO("Current position is considered as an occupied node! Finding closest unoccupied node position to consider as Start node.\n");
 }
 if (sqrt(pow((Trans_Matrix(0,3) - x_dest),2) + pow((Trans_Matrix(1,3) - y_dest),2) + pow((Trans_Matrix(2,3) - z_dest),2)) < threshold)
 {
  ROS_INFO("Destination is within threshold. Publishing stopped");
  is_dest_set = false;
  return;
 }
  int start[3] = {start_x,start_y,start_z};
  int target[3] = {dest_rel_x,dest_rel_y,dest_rel_z};
  jps.Jump_Point_Search(start,target);
  Visualiser(jps.rpath);
}
 

void JPSNodelet::pointcloud_subscriberCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
{ 
  if (!msg) {
  ROS_INFO("Did not receive pointcloud message!\n");
   return;
  }
  map_msg = msg;
  if (!(is_dest_set || is_waypoint_set))
  {return;}
  if(is_waypoint_set && !goal_waypoints.poses.empty() && !is_dest_set)
  {
    geometry_msgs::PoseStamped waypoint = goal_waypoints.poses.front();
    goal_waypoints.poses.erase(goal_waypoints.poses.begin());
    x_dest = int(waypoint.pose.position.x);
    y_dest = int(waypoint.pose.position.y);
    z_dest = int(waypoint.pose.position.z);
    ROS_INFO("Now moving to waypoint x: %d, y: %d, z: %d\n",x_dest,y_dest,z_dest);
    is_dest_set = true;
  }
  if (sqrt(pow((Trans_Matrix(0,3) - x_dest),2) + pow((Trans_Matrix(1,3) - y_dest),2) + pow((Trans_Matrix(2,3) - z_dest),2)) < threshold)
 {
  is_dest_set = false;
  if(goal_waypoints.poses.empty())
  {
    ROS_INFO("Final destination is within threshold. Path planning stopped");
    is_waypoint_set = false;
    return;
  }
 }
 counter++;
 if(counter%5 != 1 || !is_dest_set)
 {return;}
  jps.clear_ogm();
  if (msg->getDataType() == "visualization_msgs/MarkerArray")
  {
    auto marker_msg = msg->instantiate<visualization_msgs::MarkerArray>();
   for (const auto& marker : marker_msg->markers[0].points)
    {
      Eigen::Vector4d vec(marker.x,marker.y,marker.z,1.0);
      Eigen::Vector4d inv_vec = inv_Trans_Matrix*vec;
      inv_vec(0) = inv_vec(0)/cell_size + offset;
      inv_vec(1) = inv_vec(1)/cell_size + offset;
      inv_vec(2) = inv_vec(2)/cell_size + offset_z;
      if ((round(inv_vec(0))>=0 && round(inv_vec(0))<=map_size-1)&&(round(inv_vec(1))>=0 && round(inv_vec(1))<=map_size-1)&&(round(inv_vec(2))>=0 && round(inv_vec(2))<=map_size-1))
      {
        jps.padding(pad_size,int(round(inv_vec(0))),int(round(inv_vec(1))),int(round(inv_vec(2))));
      }
    }
  }

  else if (msg->getDataType() == "sensor_msgs/PointCloud2")
  {
    auto cloud_msg = msg->instantiate<PointCloud>();
    for (int i = 0; i < cloud_msg->points.size(); i++)
  {
   Eigen::Vector4d vec(cloud_msg->points[i].x,cloud_msg->points[i].y,cloud_msg->points[i].z,1.0);
   Eigen::Vector4d inv_vec = inv_Trans_Matrix*vec;
   int inv_vec_x = int(round(inv_vec(0)))/cell_size + offset;
   int inv_vec_y = int(round(inv_vec(1)))/cell_size + offset;
   int inv_vec_z = int(round(inv_vec(2)))/cell_size + offset_z;
   if ((inv_vec_x>=0 && inv_vec_x<=map_size-1)&&(inv_vec_y>=0 && inv_vec_y<=map_size-1)&&(inv_vec_z>=0 && inv_vec_z<=map_size-1))
   {
    jps.padding(pad_size,inv_vec_x,inv_vec_y,inv_vec_z);
   }
  }
  }

  else 
  {
    ROS_INFO("Callback message is not of type visualization_msgs::MarkerArray or sensor_msgs::PointCloud2. Unable to construct occupancy grid map!\n");
    return;
  }
  JPS_Publisher(x_dest,y_dest,z_dest);
}

bool JPSNodelet::service_callback(JPS::jps_service_message::Request &req, JPS::jps_service_message::Response &res)
{
  if(!odom_msg)
  {
    ROS_INFO("Did not receive Odometry information!!\n");
    return false;
  }
  if(!map_msg)
  {
    ROS_INFO("Did not receive Pointcloud information!!\n");
    return false;
  }
  if(!req.start)
  {
    is_dest_set = false;
    is_waypoint_set = false;
    res.status = "Stopping Path planning. Clearing waypoint queue.\n";
    goal_waypoints.poses.clear();
    return true;
  }
  x_dest = req.x;
  y_dest = req.y;
  z_dest  = req.z;
  is_dest_set = true;
  is_waypoint_set = false;
  odom_msg = nullptr;
  map_msg = nullptr;

    res.status = "Received new destination coordinates!\n";
    return true;
}

bool JPSNodelet::waypoint_service_callback(JPS::jps_waypoint_service::Request &req, JPS::jps_waypoint_service::Response &res)
{
  if(!odom_msg)
  {
    ROS_INFO("Did not receive Odometry information!!\n");
    return false;
  }
  if(!map_msg)
  {
    ROS_INFO("Did not receive Pointcloud information!!\n");
    return false;
  }
  goal_waypoints = req.waypoint;
  is_waypoint_set = true;
  is_dest_set = false;
  res.status = "Received new waypoints!\n";
  return true;

}


int main( int argc, char** argv)
{
  ros::init(argc, argv, "jps_server");
  JPSNodelet JPSNode;
  ROS_INFO("JPS Server Ready\n");
  ros::spin();
}