#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <typeinfo>


#include <chrono>
#include <algorithm>
#include <sys/stat.h>
#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>  

#include <opencv2/objdetect/objdetect.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/core/core.hpp>

#include <std_msgs/String.h>  

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <mutex>
#include <string>


#include <ctime>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace sensor_msgs;
using namespace message_filters;

using namespace std;
using namespace cv ; 

ofstream tsfile;

int frame_id = 0 ;  

string gDir("/home/jin/Data_Capture/myImage");
string gDataName("OBS");


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 
const double camera_factor = 1000;

// double camera_cx = 323.09356689453125;
// double camera_cy = 237.54339599609375;
// double camera_fx = 618.469482421875;
// double camera_fy = 618.7632446289062; 

// double camera_cx = 322.1780700683594;
// double camera_cy = 236.56320190429688;
// double camera_fx = 385.3685607910156;
// double camera_fy = 385.3685607910156; 


double camera_cx = 427.0935974121094;
double camera_cy = 237.54339599609375;
double camera_fx = 618.469482421875;
double camera_fy = 618.7632446289062; 

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

void ImageGrabber(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD_reg, const sensor_msgs::ImageConstPtr& msgD_raw, const sensor_msgs::ImageConstPtr& msgIr1, const sensor_msgs::ImageConstPtr& msgIr2)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD_reg;
    cv_bridge::CvImageConstPtr cv_ptrD_raw;
    cv_bridge::CvImageConstPtr cv_ptrIr1; 
    cv_bridge::CvImageConstPtr cv_ptrIr2; 

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
        cv_ptrD_reg = cv_bridge::toCvShare(msgD_reg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptrD_raw = cv_bridge::toCvShare(msgD_raw, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptrIr1 = cv_bridge::toCvShare(msgIr1, sensor_msgs::image_encodings::TYPE_16UC1); 
        cv_ptrIr2 = cv_bridge::toCvShare(msgIr2, sensor_msgs::image_encodings::TYPE_16UC1); 
        ROS_WARN("Receive msg rgb w %d h %d, dpt w %d h %d ir w %d h %d!", cv_ptrRGB->image.cols, cv_ptrRGB->image.rows, cv_ptrRGB->image.cols, cv_ptrRGB->image.rows, cv_ptrIr1->image.cols, cv_ptrIr1->image.rows); 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("Live Feed RGB:", cv_ptrRGB->image); 
    // imshow("Live Feed depth_reg:", cv_ptrD_reg->image); 
    waitKey(3); 
    
    // write images 
    stringstream tt;
    tt << msgRGB->header.stamp ;
    string time_stamp = tt.str();

    imwrite( gDir + "/" + gDataName + "/color/" + time_stamp+".png", cv_ptrRGB->image);
    imwrite( gDir + "/" + gDataName + "/depth_reg/" + time_stamp+".png", cv_ptrD_reg->image);
    imwrite( gDir + "/" + gDataName + "/depth_raw/" + time_stamp+".png", cv_ptrD_raw->image);
    imwrite( gDir + "/" + gDataName + "/ir1/" + time_stamp+".png", cv_ptrIr1->image);
    imwrite( gDir + "/" + gDataName + "/ir2/" + time_stamp+".png", cv_ptrIr2->image);

    std_msgs::Header h_rgb = msgRGB->header;

    stringstream ss;
    ss << time_stamp << "  "<< "color/"<< h_rgb.stamp << ".png  "<<h_rgb.stamp << " depth_reg/"<< h_rgb.stamp  << ".png "<<h_rgb.stamp  << " depth_raw/"<< h_rgb.stamp  << ".png "<< h_rgb.stamp <<" ir1/"<<h_rgb.stamp <<".png "<<h_rgb.stamp <<" ir2/"<<h_rgb.stamp <<".png";
    
    // ss << time_stamp << "  "<< "color/"<< time_stamp << ".png  "<<time_stamp << " depth/"<< time_stamp << ".png ";

    string time_info = ss.str();
    tsfile << time_info <<"\n";


    cv::Mat rgb, depth;

    rgb = cv_ptrRGB->image;
    depth = cv_ptrD_reg->image;

    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];


            // if (d > 600 || d < 200 )
            //     continue;
            if (d == 0)
                continue;
            PointT p;

            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // if (d > 600 || d < 200 )
            //     p.x = p.y = p.z = bad_point;
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;

    
    viewer.showCloud (cloud);

    cloud->points.clear();

    return; 
}




int main(int argc, char* argv[])
{
  ros::init(argc, argv, "save_data"); 
  ros::start(); 
  ros::NodeHandle nh; 

  ros::NodeHandle np("~"); 
  np.param("data_dir", gDir, gDir); 
  np.param("data_name", gDataName, gDataName); 

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  stringstream oss;
  oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");

  gDataName = oss.str();

  string d_dir = gDir +"/"+ gDataName; 
  string d_rgb = d_dir + "/color"; 
  string d_D_reg = d_dir + "/depth_reg";
  string d_D_raw = d_dir + "/depth_raw"; 
  string d_ir1 = d_dir + "/ir1"; 
  string d_ir2 = d_dir + "/ir2"; 

  mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  mkdir(d_rgb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_D_reg.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_D_raw.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_ir1.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_ir2.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  string f_time = d_dir + "/timestamp.txt"; 
  tsfile.open(f_time.c_str());
  // tsfile << "index  timestamp\n";

  int q = 7; 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_rect_color", q);

  message_filters::Subscriber<sensor_msgs::Image> depth_rig_sub(nh, "/camera/aligned_depth_to_color/image_raw", q); 

  message_filters::Subscriber<sensor_msgs::Image> depth_raw_sub(nh, "/camera/depth/image_rect_raw", q);

  message_filters::Subscriber<sensor_msgs::Image> ir1_sub(nh, "/camera/infra1/image_rect_raw", q); 
  message_filters::Subscriber<sensor_msgs::Image> ir2_sub(nh, "/camera/infra2/image_rect_raw", q); 

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(q), rgb_sub, depth_rig_sub, depth_raw_sub, ir1_sub, ir2_sub);

  sync.registerCallback(boost::bind(&ImageGrabber,_1, _2, _3, _4, _5));
  ROS_WARN("realsense_wrapper.cpp: start to subscribe msgs!"); 

  ros::spin(); 
  ros::shutdown(); 

  return 0;
}








