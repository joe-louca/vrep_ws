//////////////////////////////////////////////////////////
// Novint Falcon Kinematics/Dynamics based on R.E.Stamper's PhD(1997)
// with some modifications
//
// Using LibniFalcon Beta 4
//
// Alastair Barrow 26/08/09


#include <iostream>
#include <string>
#include <cmath>
#include <math.h>       /* sqrt */
#include <stdlib.h>     /* abs */


#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"

#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

FalconDevice falcon;

ros::Publisher twist_pub;

std::array<double, 3> pos;
double fx, fy, fz, tx, ty, tz;
std::array<double, 6> force;
std::array<double, 3> force_apply;
bool rpy_mode;


/// Ask libnifalcon to get the Falcon ready for action
bool initialise()
{
  falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
  cout << "Setting up comm interface for Falcon comms" << endl;
  unsigned int count;
  falcon.getDeviceCount(count);
  cout << "Connected Device Count: " << count << endl;

  //Open the device number:
  int deviceNum = 0;
  cout << "Attempting to open Falcon device:  " << deviceNum << endl;
	
  if(!falcon.open(deviceNum))
  {
    cout << "Cannot open falcon device index " << deviceNum << " - Lib Error Code: " << falcon.getErrorCode() << " Device Error Code: " << falcon.getFalconComm()->getDeviceErrorCode() << endl;
    return false;
  }
  else
  {
    cout << "Connected to Falcon device " << deviceNum << endl ;
  }

  //Load the device firmware:
  falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
  bool skip_checksum = false;
  bool firmware_loaded = false;
  firmware_loaded = falcon.isFirmwareLoaded();
  if(!firmware_loaded)
  {
    std::cout << "Loading firmware" << std::endl;
    uint8_t* firmware_block;
    long firmware_size;
    firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
    firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;

    for(int i = 0; i < 10; ++i)
    {
      if(!falcon.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
      {cout << "Firmware loading try failed";}
      else
      {firmware_loaded = true; break;}
    } 
  }
  
  else if(!firmware_loaded)
  {std::cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << std::endl;}

  if(!firmware_loaded || !falcon.isFirmwareLoaded())
  {std::cout << "No firmware loaded to device, cannot continue" << std::endl;}
  std::cout << "Firmware loaded" << std::endl;

  //Seems to be important to run the io loop once to be sure of sensible values next time:
  falcon.runIOLoop();

  falcon.getFalconFirmware()->setHomingMode(true);
  falcon.setFalconKinematic<FalconKinematicStamper>();
  falcon.setFalconGrip<libnifalcon::FalconGripFourButton>();

  return true;
}


void publish_twist(float deadzone, std::array<double, 3>& falcon_pos, bool falcon_rpy_mode, string grip_ctrl)
{
  // Publish x, y, z as twist. Either linear or angular input
  geometry_msgs::TwistStamped twist;
  twist.header.stamp = ros::Time::now();
  twist.header.frame_id = grip_ctrl;
  
  if (!falcon_rpy_mode)
  {
    twist.twist.linear.x = (abs(falcon_pos[0]) > deadzone) ? falcon_pos[0] : 0; // X (left/right) // if falcon position greater than deadzone, publish position, else publish 0
    twist.twist.linear.y = (abs(falcon_pos[2]) > deadzone) ? -falcon_pos[2] : 0;// Y (in/out)
    twist.twist.linear.z = (abs(falcon_pos[1]) > deadzone) ? falcon_pos[1] : 0; // Z (up/down)
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;  
  }
  else
  {
    twist.twist.linear.x = 0.0;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = 0.0;
    twist.twist.angular.x = (abs(falcon_pos[0]) > deadzone) ? -falcon_pos[0] : 0; // pitch (up/down)
    twist.twist.angular.y = (abs(falcon_pos[1]) > deadzone) ? -falcon_pos[1] : 0; // yaw (left/right
    twist.twist.angular.z = (abs(falcon_pos[2]) > deadzone) ? falcon_pos[2] : 0; // roll (in/out)
  }
  twist_pub.publish(twist);
}


void ft_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  std::cout << "in callback" << std::endl;
  if (!rpy_mode)
  {
    std::cout << "reading force" << std::endl;
    force[0] = msg->data[0];
    force[1] = msg->data[1];
    force[2] = msg->data[2];
  }
  else
  {
    std::cout << "reading t" << std::endl;
    force[0] = msg->data[3];
    force[1] = msg->data[4];
    force[2] = msg->data[5];
  }
    
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "falcon_stream");
  ros::NodeHandle n;
  ros::Subscriber ft_sub;

  twist_pub = n.advertise<geometry_msgs::TwistStamped>("falcon/twist", 1);

  if(!initialise())
  {
    return 0;
  }
  
  // Get mid positions
  float z_min = 0.0745;
  float z_max = 0.175;
  float z_mid = z_min + (z_max - z_min)/2.0;

  float x_min = -0.053;
  float x_max = 0.053;
  float x_mid = x_min + (x_max - x_min)/2.0;

  float y_min = -0.053;
  float y_max = 0.053;
  float y_mid = y_min + (y_max - y_min)/2.0;
  
  // Get threshold values
  float f_threshold;
  float t_threshold;
  n.getParam("f_threshold",f_threshold);
  n.getParam("t_threshold",t_threshold);

  int max_falcon_force = 75; // 100: ok
  int min_falcon_force = 3;
  float f_scale_factor = f_threshold/min_falcon_force;
  float t_scale_factor = t_threshold/min_falcon_force;  
  
  // Initialise button control variables	
  int btn;
  string gripper_control;

  // Set loop rate
  int rate_hz;
  n.getParam("rate_hz",rate_hz);
  ros::Rate loop_rate(rate_hz);

  std::array<double, 3> oldpos;
  oldpos[0] = oldpos[1] = oldpos[2] = 0.0;
  
  while(ros::ok())
  {
    ///// CONTROLLER INPUT /////
    
    // Set button variables
    rpy_mode = false;
    gripper_control = "n";
    
    //Ask libnifalcon to update the encoder positions and apply any forces waiting:
    falcon.runIOLoop();
    pos = falcon.getPosition();
    //std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    // Normalise outputs
    pos[0] = pos[0]*1.0/(x_max-x_mid);
    pos[1] = pos[1]*1.0/(y_max-y_mid);
    pos[2] = (pos[2]-z_mid)*1.0/(z_max-z_mid);
    //std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
    
    // If centre button pressed, switch to rpy mode
    btn = falcon.getFalconGrip()->getDigitalInputs(); 
    if (btn & libnifalcon::FalconGripFourButton::CENTER_BUTTON)
    {
      rpy_mode = true;
    }
    
    // read gripper button
    if (btn & libnifalcon::FalconGripFourButton::FORWARD_BUTTON)
    {
      gripper_control = "y";
    }
    else 
    {
      gripper_control = "n";
    }
    
    // Publish controller input
    publish_twist(0.15, pos, rpy_mode, gripper_control); 


    ///// FORCE FEEDBACK /////

    // Get ft reading
    //ft_sub = n.subscribe<std_msgs::Float32MultiArray>("delayed_ft", 1, ft_callback);     
    n.getParam("ft_delay/fx",fx);
    n.getParam("ft_delay/fy",fy);
    n.getParam("ft_delay/fz",fz);
    n.getParam("ft_delay/tx",tx);
    n.getParam("ft_delay/ty",ty);
    n.getParam("ft_delay/tz",tz);
    
     
    // Spring force back to Falcon home position
    double stiffness = 100.0;
    double eucdist2centre = sqrt(pos[0]*pos[0]+pos[1]*pos[1]+pos[2]*pos[2]); 
    
    if (abs(fx) < f_threshold)
    {
      stiffness = 100.0;
      force_apply[0] = -stiffness*eucdist2centre * (pos[0]*pos[0]*pos[0]);// + fx/100.0;// + fx*pos[0]; // -ve pushes Falcon Left / +ve pushes Falcon Right (x axis) (10 is quite strong)
      force_apply[1] = -stiffness*eucdist2centre * (pos[1]*pos[1]*pos[1]);// + fz/40.0;//0;//-min_falcon_force*pos[1]; // -ve pushes Falcon Down / +ve pushes Falcon Up (z axis)
      force_apply[2] = -stiffness*eucdist2centre * (pos[2]*pos[2]*pos[2]);// + fy/40.0;//0;//-min_falcon_force*pos[2]; // -ve pushes Falcon In / +ve pushes Falcon Out (y axis)
      std::cout << "below thresh" << std::endl;
    }
    else
    {
      stiffness = 100.0;
      force_apply[0] = -stiffness* (pos[0]*pos[0]*pos[0]);// + fx/100.0;// + fx*pos[0]; // -ve pushes Falcon Left / +ve pushes Falcon Right (x axis) (10 is quite strong)
      force_apply[1] = -stiffness* (pos[1]*pos[1]*pos[1]);// + fz/40.0;//0;//-min_falcon_force*pos[1]; // -ve pushes Falcon Down / +ve pushes Falcon Up (z axis)
      force_apply[2] = -stiffness* (pos[2]*pos[2]*pos[2]);// + fy/40.0;//0;//-min_falcon_force*pos[2]; // -ve pushes Falcon In / +ve pushes Falcon Out (y axis)
    }
    
    // Switch based on XYZ or RPY control modes
    if (!rpy_mode)
    { 
      int test = 1;
      // Force feedback relative to force sensor readings  
      // For each axis, if F above max threshold, set to max force
      //if (fabs(fx)>max_falcon_force*f_scale_factor)
      //  {force_apply[0] = -max_falcon_force*fabs(fx)/fx;} 	// max force in opposite direction to ft reading
      // Else set to scaled force
      //else if (fabs(fx)>f_threshold)
      //  {force_apply[0] = fx/f_scale_factor;}	 	// scaled force in opposite direction to ft reading

      //if (fabs(fz)>max_falcon_force*f_scale_factor)
      //  {force_apply[1] = -max_falcon_force*fabs(fz)/fz;}
      //else if (fabs(fz)>f_threshold)
      //  {force_apply[1] = fz/f_scale_factor*-pos[1];}
        
      //if (fabs(fy)>max_falcon_force*f_scale_factor)
      //  {force_apply[2] = -max_falcon_force*fabs(fy)/fy;}	
      //else if (fabs(fy)>f_threshold)
      //  {force_apply[2] = -fy/f_scale_factor;}
               	 
    }
    //else 
    //{
      // Else do the same with torque
      //if (fabs(tx)>max_falcon_force*t_scale_factor)
      //  {force_apply[0] = -max_falcon_force*fabs(tx)/tx;} 	
      //else if (fabs(tx)>t_threshold)
      //  {force_apply[0] = -tx/t_scale_factor;}	 	
        
      //if (fabs(ty)>max_falcon_force*t_scale_factor)
      //  {force_apply[1] = -max_falcon_force*fabs(ty)/ty;} 	
      //else if (fabs(ty)>t_threshold)
      //  {force_apply[1] = -ty/t_scale_factor;}	 	
        
      //if (abs(tz)>max_falcon_force*t_scale_factor)
      //  {force_apply[2] = -max_falcon_force*fabs(tz)/tz;} 	
      //else if (fabs(tz)>t_threshold)
      //  {force_apply[2] = -tz/t_scale_factor;}
    //}
    
    // dampen position update to remove noise.
    oldpos[0] = 0.4*oldpos[0]+ 0.6*pos[0];
    oldpos[1] = 0.4*oldpos[1]+ 0.6*pos[1];
    oldpos[2] = 0.4*oldpos[2]+ 0.6*pos[2];
    
    falcon.setForce(force_apply);
    
    loop_rate.sleep();    
  }
  
  falcon.close();

  return 0;
}


