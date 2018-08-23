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

using namespace sensor_msgs;
using namespace message_filters;

using namespace std;
using namespace cv ; 

ofstream tsfile;

int frame_id = 0 ;  

string gDir("/home/jin/Data_Capture/myImage");
string gDataName("OBS");

void ImageGrabber(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgAD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrAD;

    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
        cv_ptrAD = cv_bridge::toCvShare(msgAD, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("Live Feed RGB:", cv_ptrRGB->image); 
    waitKey(3); 
    
    // write images 
    stringstream tt;
    tt << msgRGB->header.stamp ;
    string tt_name = tt.str();
    string time_stamp = tt_name.substr (0,17);

    imwrite( gDir + "/" + gDataName + "/color/" + time_stamp+".png", cv_ptrRGB->image);
    // imwrite( gDir + "/" + gDataName + "/aligned_depth/" + tt_name+".png", cv_ptrAD->image);
    imwrite( gDir + "/" + gDataName + "/depth/" + time_stamp+".png", cv_ptrAD->image);
    // imwrite( gDir + "/" + gDataName + "/ir/" + tt_name+".png", cv_ptrIr1->image);
    // imwrite( gDir + "/" + gDataName + "/ir2/" + tt_name+".png", cv_ptrIr2->image);

    stringstream ss;
    
    ss << time_stamp << "  "<< "color/"<< time_stamp << ".png  "<<time_stamp << " depth/"<< time_stamp << ".png ";

    string time_info = ss.str();
    tsfile << time_info <<"\n";

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

  string d_dir = gDir +"/"+ gDataName; 
  string d_rgb = d_dir + "/color"; 
  // string d_Adpt = d_dir + "/aligned_depth";
  string d_dpt = d_dir + "/depth"; 
  // string d_ir = d_dir + "/ir"; 
  // string d_ir2 = d_dir + "/ir2"; 

  mkdir(d_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_Adpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_rgb.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir(d_dpt.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  // mkdir(d_ir2.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  string f_time = d_dir + "/timestamp.txt"; 
  tsfile.open(f_time.c_str());
  // tsfile << "index  timestamp\n";

  int q = 7; 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", q);

  message_filters::Subscriber<sensor_msgs::Image> Adepth_sub(nh, "/camera/depth_registered/image_raw", q); 

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(q), rgb_sub, Adepth_sub);

  sync.registerCallback(boost::bind(&ImageGrabber,_1, _2));




  ROS_WARN("realsense_wrapper.cpp: start to subscribe msgs!"); 

  ros::spin(); 
  ros::shutdown(); 

  return 0;
}








