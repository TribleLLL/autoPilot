#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h> 
#include <iostream>
#include <cstring>
#include <list>
#include <math.h>
// #include <std_msgs>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

using std::endl;
using std::list;
using std::cout;
geometry_msgs::PoseWithCovarianceStamped* _initpose;
geometry_msgs::PoseStamped* _goalpose;
nav_msgs::Path* _path;

ros::Publisher pub;
ros::Publisher pub_path;

float minx, miny;
int cof;
int width,height;
bool map_obs[271*3][361*3];


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	/*// Create a container for the data.
	sensor_msgs::PointCloud2 output;

	// Do data processing here...
	output = *input;*/

	// std::cout<<"test3\n";
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	pcl_conversions::toPCL(*input, *cloud);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize(0.02f,0.02f,0.02f);
	sor.filter(cloud_filtered);

	sensor_msgs::PointCloud2 output;

	nav_msgs::OccupancyGrid occupancyGrid;


	pcl_conversions::fromPCL(cloud_filtered, output);
	// std::cout<<"test2\n";

	/*****/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(output, *cloud2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 output2;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

	coefficients->values.resize(4);

	coefficients->values[0] = 0;

	coefficients->values[1] = 0;

	coefficients->values[2] = 1;

	coefficients->values[3] = 0;


	// Create the filtering object  
	pcl::PassThrough<pcl::PointXYZ> pass;  

	pass.setInputCloud (cloud2);  

	pass.setFilterFieldName ("z");  

	pass.setFilterLimits (-0.4, 11);

	//pass.setFilterLimitsNegative (true);  
	pass.filter (*cloud2);


	// build the filter
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

	outrem.setInputCloud(cloud2);

	outrem.setRadiusSearch(0.1);

	outrem.setMinNeighborsInRadius (5);

	// apply filter
	outrem.filter (*cloud2);



	// Create the filtering object

	pcl::ProjectInliers<pcl::PointXYZ> proj;//创建投影滤波对象

	proj.setModelType(pcl::SACMODEL_PLANE);//设置对象对应的投影模型

	proj.setInputCloud(cloud2);//设置输入点云

	proj.setModelCoefficients(coefficients);//设置模型对应的系数

	proj.filter(*cloud_projected);//执行投影滤波存储结果cloud_projected

	pcl::toROSMsg(*cloud_projected, output2);

	// pub.publish (output2);


	// for (int i = 0; i < 1; i++)
	//  std::cout << i << " OUT XYZ:" << cloud_projected->points[i].x 
	//      << " " << cloud_projected->points[i].y << " "
	//      << cloud_projected->points[i].z << std::endl;

	// std::cout << "height " << cloud_projected->height << std::endl;
	// std::cout << "width " << cloud_projected->width << std::endl;
	// std::cout << "is_dense " << cloud_projected->is_dense << std::endl;
	// std::cout << "points.size " << cloud_projected->size() << std::endl;
	float  maxx, maxy;
	minx = maxx = cloud_projected->points[0].x;
	miny = maxy = cloud_projected->points[0].y;
	for (int i=0; i<cloud_projected->size();i++){
		if (cloud_projected->points[i].x > maxx) maxx = cloud_projected->points[i].x;
		if (cloud_projected->points[i].x < minx) minx = cloud_projected->points[i].x;
		if (cloud_projected->points[i].y > maxy) maxy = cloud_projected->points[i].y;
		if (cloud_projected->points[i].y < miny) miny = cloud_projected->points[i].y;
	}

	cof = 3;
	width = 270 * cof;
	height = 360 * cof;

    int map_2d[width+1][height+1];
	memset(map_2d,0,sizeof(map_2d));
	for (int i=0; i<cloud_projected->size();i++){
		map_2d[(int)((cloud_projected->points[i].x - minx)*cof*10)][(int)((cloud_projected->points[i].y- miny)*cof*10)]++;
	}

	occupancyGrid.header.seq = cloud_projected->header.seq;
	// occupancyGrid.header.stamp = cloud_projected->header.stamp;
	occupancyGrid.header.frame_id = cloud_projected->header.frame_id;
	occupancyGrid.info.resolution = 0.1/cof;
	occupancyGrid.info.width = width;
	occupancyGrid.info.height = height;
	// occupancyGrid.info.origin.position.x = cloud_projected->sensor_origin_.x ;
	// occupancyGrid.info.origin.position.y = cloud_projected->sensor_origin_.y ;
	occupancyGrid.info.origin.position.x = 0;
	occupancyGrid.info.origin.position.y = 0;
	occupancyGrid.info.origin.position.z = 0;
	occupancyGrid.info.origin.orientation.x = 0;
	occupancyGrid.info.origin.orientation.y = 0; 
	occupancyGrid.info.origin.orientation.z = 0.0;
	occupancyGrid.info.origin.orientation.w = 1.0;
	occupancyGrid.data.resize(width*height);
	// memset(occupancyGrid.data, 0, sizeof(occupancyGrid.data) );
	int index = 0;
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++){
			// if (map_2d[x][y] == 0) 
			//  occupancyGrid.data[index++] = 0;
			// else if (map_2d[x][y] == 1) 
			//  occupancyGrid.data[index++] = 20;
			// else if (map_2d[x][y] == 2) 
			//  occupancyGrid.data[index++] = 40;
			// else if (map_2d[x][y] == 3) 
			//  occupancyGrid.data[index++] = 60;
			// else if (map_2d[x][y] == 4) 
			//  occupancyGrid.data[index++] = 80;
			// else       
			//  occupancyGrid.data[index++] = 100;

		  	if (map_2d[x][y] > 0) {
		    	occupancyGrid.data[index++] = 100;
          map_obs[x][y] = 1;
        }
		  	else
		    	occupancyGrid.data[index++] = 0;
		}


	// Publish the data.
	pub.publish (occupancyGrid);
	if (_path != NULL)
	 	pub_path.publish (*_path);
}

struct Point {
  	int x,y;
	float F,G,H;
	// bool inOpenList;
	// bool inCloseList;
	Point* father;
	Point() {
		x = y = F = G = H = -1;
		// inOpenList = inCloseList = false;
		father = NULL;
	}
};
typedef list<Point*> list_PointPtr;

// Point _MAP_2D[500][500];
list_PointPtr openlist;
list_PointPtr closelist;
Point start,dest;

Point* isInList(std::list<Point*> &list, Point* point)
{  
    //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标  
    for(list_PointPtr::iterator p = list.begin(); p != list.end(); p++)  
        if((*p)-> x == point -> x && (*p)-> y == point -> y)  
            return (*p);  
    return NULL;  
}  

Point* getLeastPoint()  
{  
    if(!openlist.empty())  
    {  
        Point* resPoint = openlist.front();  
        for(list_PointPtr::iterator point = openlist.begin(); point != openlist.end(); point++)  
            if((*point)->F < resPoint->F)  
                resPoint = (*point);  
        return resPoint;  
    }  
    return NULL;  
}  

void updateNeighbour(Point *present) {
	Point nb[4];
	nb[0].x = present->x + 1; nb[0].y = present->y;
	nb[1].x = present->x - 1; nb[1].y = present->y;
	nb[2].x = present->x;	  nb[2].y = present->y + 1;
	nb[3].x = present->x;     nb[3].y = present->y - 1;
  // cout << present->x << "," << present->y << endl;
	for (int i = 0; i < 4; i++)
	{
		nb[i].G = present->G + 10;
		nb[i].H = abs(dest.x - nb[i].x ) + abs(dest.y - nb[i].y);
    nb[i].F = nb[i].G + nb[i].H;
    nb[i].father = present;
    // cout << nb[i].father->x << "," << nb[i].father->y << endl;
    //it is not a obstruction
		if (map_obs[nb[i].x][nb[i].y] != 1){
			// it is not in closelist
			if (isInList(closelist, &nb[i]) == NULL){		

				if (isInList(openlist, &nb[i]) == NULL){
					Point* temp = new Point;
					*temp = nb[i];
					openlist.push_back(temp);
				}else{
					Point* temp = isInList(openlist, &nb[i]);
					if (temp->F > nb[i].F)
						*temp = nb[i];					
				}
			}
		}
	}

}


void drawPath(){
/**A****/
  cout << "begin drawPath \n";
	start.x = (int)((_initpose->pose.pose.position.x - minx)*cof*10);
	start.y = (int)((_initpose->pose.pose.position.y - miny)*cof*10);

	dest.x = (int)((_goalpose->pose.position.x - minx)*cof*10);
	dest.y = (int)((_goalpose->pose.position.y - miny)*cof*10);

	start.G = 0;
	start.F = start.H =  abs(dest.x - start.x ) + abs(dest.y - start.y);
  start.father = NULL;

  cout << "initial\n";
	// openlist.clear();
	// closelist.clear(); 
  	_path = new nav_msgs::Path();
    (*_path).header.frame_id="odom";
  cout << "clear\n";

	openlist.push_back(&start);

  cout << "push start point\n";

	Point* present;
	while(isInList(openlist, &dest) == NULL){
    // cout << "1\n";
		present = getLeastPoint();
		openlist.remove(present);
		closelist.push_back(present);
		updateNeighbour(present);
	}

  cout << "find the dest\n";
	
	present = isInList(openlist, &dest);	
	// cout << present->father->x << " , " <<  present->father->y << endl;
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id="odom";
  cout << "2"<<endl;
  int count = 0;
  while (present != NULL){
    cout << "num " << count << endl;
    count++;
    if(present == &start) break;
    present = present->father;

    // cout << present->x << " , " <<  present->y << endl;
  }

  cout << "finish???" << endl;

  cout << "count1 = " << count << endl;

  // _path->poses.resize(count+1);

  cout << "count2 = "<<count<<endl;

  present = isInList(openlist, &dest);
  int ct = 0;
	while (present != NULL){		
    cout<<"3"<<endl;

    // _path->poses[ct].pose.position.x = float(present->x) /cof /10 + minx;
    // _path->poses[ct].pose.position.y = float(present->y) /cof /10 + miny;
    // _path->poses[ct].pose.position.z = 0;
    // _path->poses[ct].header.frame_id="odom"; 
    // ct++;
    // cout<<"4"<<endl;      
    // // _path->poses.push_back(this_pose_stamped);
    // cout << "point "<< present->x << ", " << present->y <<endl;
	geometry_msgs::PoseStamped this_pose_stamped;
	cout<<"4.1"<<endl;     
	this_pose_stamped.pose.position.x = float(present->x) /cof /10 + minx;
	cout<<"4.2"<<endl;     
	this_pose_stamped.pose.position.y = float(present->y) /cof /10 + miny;
	cout<<"4.3"<<endl;     
	this_pose_stamped.pose.position.z = 0;
	cout<<"4.4"<<endl;     
	this_pose_stamped.header.frame_id = "odom";
	cout<<"4.5"<<endl;     
	(*_path).poses.push_back(this_pose_stamped);
	cout<<"4.6"<<endl;     
    present = present->father;
	}
  cout << "finish drawPath\n";



  


// /******/
// 	if (_initpose != NULL && _goalpose != NULL){    
// 		float d_x = (_goalpose->pose.position.x - _initpose->pose.pose.position.x)/100;
// 		float d_y = (_goalpose->pose.position.y - _initpose->pose.pose.position.y)/100;

// 		if (_path==NULL)
// 		    _path = new nav_msgs::Path();
// 		    _path->poses.resize(101);
// 		    _path->header.frame_id = "odom";
// 		geometry_msgs::PoseStamped this_pose_stamped; 

// 		for (int i = 0; i <= 100 ; i++){
// 			_path->poses[i].pose.position.x = _initpose->pose.pose.position.x + d_x * i;
// 			_path->poses[i].pose.position.y = _initpose->pose.pose.position.y + d_y * i;
// 			_path->poses[i].pose.position.z = 0;
// 			_path->poses[i].header.frame_id="odom";		

// 		}   
// 	}
}

void 
initialpose (const geometry_msgs::PoseWithCovarianceStamped& input){
	_initpose = new geometry_msgs::PoseWithCovarianceStamped();
	*_initpose = input;
	std::cout << "initialpose" << _initpose->pose.pose.position.x << ", "
				<< _initpose->pose.pose.position.y << ", "
				<< _initpose->pose.pose.position.z << "\n ";        
	if (_initpose != NULL && _goalpose != NULL)
    drawPath();
}

void 
goal (const geometry_msgs::PoseStamped& input){
	_goalpose = new geometry_msgs::PoseStamped();
	*_goalpose = input;
	std::cout << "goalpose" << _goalpose->pose.position.x << ", "
				<< _goalpose->pose.position.y << ", "
				<< _goalpose->pose.position.z << "\n ";
	if (_initpose != NULL && _goalpose != NULL)
  drawPath();
}



int
main (int argc, char** argv)
{
	// Initialize ROS
	std::cout<<"bein4\n";
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	ros::NodeHandle nh2;

	_initpose = NULL;
	_goalpose = NULL;
	_path = NULL;

  memset(map_obs,0,sizeof(map_obs));

	pub = nh.advertise<nav_msgs::OccupancyGrid> ("output", 1);

	// pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

	pub_path = nh2.advertise<nav_msgs::Path> ("output_path", 1);

	ros::Subscriber sub = nh.subscribe ("point_cloud", 1, cloud_cb);

	ros::Subscriber sub_initialpose = nh.subscribe ("initialpose", 1, initialpose);

	ros::Subscriber sub_goal = nh.subscribe ("move_base_simple/goal", 1,  goal);

 
  
	// Spin
	ros::spin ();
}
