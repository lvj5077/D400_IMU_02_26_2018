#include <stdio.h>
#include <unistd.h>
#include <iomanip>
#include <typeinfo>
#include <sys/stat.h>

#include <ros/ros.h>
#include <fstream>
#include <string>
#include <cmath>
#include "vectornav.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std; 

#define D2R(d) ((d*M_PI)/180.)

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;
static int cnt = 0; 

void asyncDataListener(
	void* sender,
	VnDeviceCompositeData* data);

int imu_record();

string fname = "/home/jin/Data_Capture/imu_vn100.log";
ofstream* pf = 0; 
ros::Publisher euler_pub; 
bool b_publish_rpy = true ;

int main(int argc, char* argv[])
{
  // mkdir("/home/jin/myIMU", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  ros::init(argc, argv, "imu_recorder"); 
  ros::NodeHandle n; 

  // init parameters 
  ros::NodeHandle np("~"); 
  np.param("imu_record_file", fname, fname); 
  np.param("publish_rpy", b_publish_rpy, b_publish_rpy); 

  if(fname != "")
  {
    pf = new ofstream(fname.c_str()); 
    if(!pf->is_open())
    {
      ROS_ERROR("%s failed to open file %s, exist", __FILE__, fname.c_str());
      pf = 0; 
    }else{
      ROS_WARN("%s try to record imu data in file %s", __FILE__, fname.c_str());
    }
  }
  if(b_publish_rpy)
  {
    ROS_INFO("publish msg '/euler_msg' ");
    euler_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); 
  }

  imu_record(); 
  
  if(pf!=0) 
  {
    pf->close(); 
    delete pf;
  }

  return 0; 
}

int imu_record()
{
	VN_ERROR_CODE errorCode;
	Vn100 vn100;

	errorCode = vn100_connect(
		&vn100,
		COM_PORT,
		BAUD_RATE);

	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {

		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");

		return 0;
	}
	else if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to connect to the sensor.\n");

		return 0;
	}

	/* Disable ASCII asynchronous messages since we want to demonstrate the
	   the binary asynchronous messages. */
	errorCode = vn100_setAsynchronousDataOutputType(
        &vn100,
        VNASYNC_OFF,
        true);

	/* Now configure the binary messages output. Notice how the configuration
	   flags can be joined using the binary OR. */
	errorCode = vn100_setBinaryOutput1Configuration(
		&vn100,
		BINARY_ASYNC_MODE_SERIAL_1,		/* Data will be output on serial port 1. This should be the one we are connected to now. */
		4,							/* Outputting binary data at 4 Hz (800 Hz on-board filter / 200 = 4 Hz). */
		BG1_YPR | BG1_ANGULAR_RATE | BG1_ACCEL,
		BG3_NONE,
		BG5_NONE,
		true);

	printf("Yaw, Pitch, Roll\n");

	/* Now register to receive notifications when a new binary asynchronous
	   packet is received. */
	errorCode = vn100_registerAsyncDataReceivedListener(&vn100, &asyncDataListener);

	/* Sleep for 10 seconds. Data will be received by the asycDataListener
	   during this time. */
        while(ros::ok())
        {
          ros::spinOnce(); 
          usleep(1000*100);  
	    // sleep(10);
        }

        // printf("fps: cnt %f = %d / 10 \n", (float)cnt/10., cnt);

	errorCode = vn100_unregisterAsyncDataReceivedListener(&vn100, &asyncDataListener);
	
	errorCode = vn100_disconnect(&vn100);
	
	if (errorCode != VNERR_NO_ERROR)
	{
		printf("Error encountered when trying to disconnect from the sensor.\n");
		
		return 0;
	}

	return 0;

}

void asyncDataListener(void* sender, VnDeviceCompositeData* data)
{
        ros::Time t = ros::Time::now(); 
	printf(" %i data: %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f\n", ++cnt, 
		data->ypr.yaw,
		data->ypr.pitch,
		data->ypr.roll, 
                data->acceleration.c0,
                data->acceleration.c1, 
                data->acceleration.c2, 
                data->angularRate.c0, 
                data->angularRate.c1, 
                data->angularRate.c2);
        if(b_publish_rpy)
        {
          std_msgs::Float32MultiArray msg; 
          msg.data.resize(3); 
          msg.data[0] = D2R(data->ypr.roll); 
          msg.data[1] = D2R(data->ypr.pitch); 
          msg.data[2] = D2R(data->ypr.yaw); 
          euler_pub.publish(msg); 
          ros::spinOnce(); 
        }

        if(pf != 0)
        {
<<<<<<< HEAD
          (*pf) << std::fixed<< std::setprecision(9)<< t.toSec()<<"\t"<< 
=======
          (*pf) << std::fixed<< t.toSec()<<"\t"<< 
>>>>>>> 04de79dfdccf72673360917b92e1c1e17e60d1a4
                data->acceleration.c0 << "\t"<<
                data->acceleration.c1 << "\t"<<
                data->acceleration.c2 << "\t"<<
                data->angularRate.c0 << "\t"<<
                data->angularRate.c1 << "\t"<<
                data->angularRate.c2 <<"\t" <<  
                data->ypr.yaw << "\t"<<
		data->ypr.pitch << "\t"<<
		data->ypr.roll<< "\t"<< endl; 
        }
}
/*
void asyncDataListener(void* sender, VnDeviceCompositeData* data)
{
	printf(" %i data: %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f %+#7.2f\n", ++cnt, 
		data->ypr.yaw,
		data->ypr.pitch,
		data->ypr.roll, 
                data->acceleration.c0,
                data->acceleration.c1, 
                data->acceleration.c2, 
                data->angularRate.c0, 
                data->angularRate.c1, 
                data->angularRate.c2);
}*/
