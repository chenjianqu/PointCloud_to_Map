#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <fstream>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;


using namespace std;

void PCLTest();
ros::Publisher pub_pcs;
ros::Publisher pub_pcr;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_test_node");//初始化节点
	ros::start();//启动节点
	
	ros::NodeHandle nh;
	pub_pcs=nh.advertise<sensor_msgs::PointCloud2>("/point_cloud",1);
	pub_pcr=nh.advertise<PointCloud>("/point_cloud_raw",1);
		
	ROS_INFO_STREAM("Initing");
	
	
	PCLTest();
	
	ROS_INFO_STREAM("pcl_test节点结束");
	return 0;
}

void PCLTest()
{

    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d> poses;         // 相机位姿
    
    ifstream fin("./data/pose.txt");
    if (!fin)
    {
        cerr<<"cannot find pose file"<<endl;
        return;
    }
    
    for ( int i=0; i<5; i++ )
    {
        boost::format fmt( "./data/%s/%d.%s" ); //图像文件格式
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        double data[7] = {0};
        for ( int i=0; i<7; i++ )
        {
            fin>>data[i];
        }
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
 
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 
    
    //pcl::visualization::CloudViewer viewer("pcd viewer");
	
    
    
    for ( int i=0; i<5; i++ )
    {
        PointCloud::Ptr current( new PointCloud );
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                if ( d >= 3500 ) continue; // 深度太大时不稳定，去掉
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;


                
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                current->points.push_back( p );
            }
            
        //利用统计滤波器方法去除孤立点。
        PointCloud::Ptr tmp ( new PointCloud );
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter( *tmp );
        (*pointCloud) += *tmp;
	
		
		//viewer.showCloud(pointCloud);
		//getchar();
    }
    getchar();
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    
    //体素滤波器（Voxel Filter）进行降采样
    pcl::VoxelGrid<PointT> voxel_filter; 
    voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       //分辨率
    PointCloud::Ptr tmp ( new PointCloud );
    voxel_filter.setInputCloud( pointCloud );
    voxel_filter.filter( *tmp );
    tmp->swap(*pointCloud);
    
    cout<<"滤波之后，点云共有"<<pointCloud->size()<<"个点."<<endl;
    
    
    sensor_msgs::PointCloud2 pmsg;
    pcl::toROSMsg(*pointCloud, pmsg);
    pmsg.header.frame_id = "map";		
    pub_pcs.publish(pmsg);
    
    pub_pcr.publish(*pointCloud);

    getchar();

  //while (!viewer.wasStopped ())
  //{
  //} 
    
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );
}

