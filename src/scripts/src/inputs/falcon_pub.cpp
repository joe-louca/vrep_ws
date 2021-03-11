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
  //There's only one kind of firmware right now, so automatically set that.
  falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
  //Next load the firmware to the device

  bool skip_checksum = false;
  //See if we have firmware
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

      {
        cout << "Firmware loading try failed";
      }
      else
      {
        firmware_loaded = true;
        break;
      }
    } 
  }
  
  else if(!firmware_loaded)
  {
    std::cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << std::endl;
  }

  if(!firmware_loaded || !falcon.isFirmwareLoaded())
  {
    std::cout << "No firmware loaded to device, cannot continue" << std::endl;
  }
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
    twist.twist.linear.x = (abs(falcon_pos[0]) > deadzone) ? falcon_pos[0] : 0;
    twist.twist.linear.y = (abs(falcon_pos[2]) > deadzone) ? falcon_pos[2] : 0; 
    twist.twist.linear.z = (abs(falcon_pos[1]) > deadzone) ? falcon_pos[1] : 0;
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;  
  }
  else
  {
    twist.twist.linear.x = 0.0;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = 0.0;
    twist.twist.angular.x = (abs(falcon_pos[0]) > deadzone) ? falcon_pos[0] : 0;
    twist.twist.angular.y = (abs(falcon_pos[1]) > deadzone) ? falcon_pos[1] : 0; 
    twist.twist.angular.z = (abs(falcon_pos[2]) > deadzone) ? falcon_pos[2] : 0;  
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

  float x_min = -0.051;
  float x_max = 0.051;
  float x_mid = x_min + (x_max - x_min)/2.0;

  float y_min = -0.053;
  float y_max = 0.053;
  float y_mid = y_min + (y_max - y_min)/2.0;
  
  // Get threshold values
  float f_threshold;
  float t_threshold;
  n.getParam("f_threshold",f_threshold);
  n.getParam("t_threshold",t_threshold);

  // Initialise button control variables	
  int btn;
  string gripper_control;

  // Set loop rate
  int rate_hz;
  n.getParam("rate_hz",rate_hz);
  //ros::Rate loop_rate(rate_hz);

  while(ros::ok())
  {
    //// CONTROLLER INPUT
    
    // Set button variables
    rpy_mode = false;
    gripper_control = "n";
    
    //Ask libnifalcon to update the encoder positions and apply any forces waiting:
    falcon.runIOLoop();
    pos = falcon.getPosition();
    
    // Normalise outputs
    pos[0] = pos[0]*1.0/(x_max-x_mid);
    pos[1] = pos[1]*1.0/(y_max-y_mid);
    pos[2] = (pos[2]-z_mid)*1.0/(z_max-z_mid);
    
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

    force_apply[0] = -7*pos[0];
    force_apply[1] = -7*pos[1];
    force_apply[2] = -7*pos[2];


    ///// FORCE FEEDBACK - TO DO
    // Get ft reading
    //ft_sub = n.subscribe<std_msgs::Float32MultiArray>("delayed_ft", 1, ft_callback);     
            
    // IF NOT AT THRESHOLD THEN SPRING FORCE TO RETURN TO HOME POSITION:
    //if (!rpy_mode)
    //{ 
    //  if (!(abs(force[0])>f_threshold || abs(force[1])>f_threshold || abs(force[2])>f_threshold)) // Force
    //  {
    //    force_apply[0] = -7*pos[0];
    //    force_apply[1] = -7*pos[1];
    //    force_apply[2] = -7*pos[2];
    //  }
    //  else
    //  {
    //    force_apply[0] = -abs(force[0])*pos[0]; // x falcon, x sim
    //    force_apply[1] = -abs(force[2])*pos[2]; // y falcon, z sim
    //    force_apply[2] = -abs(force[1])*pos[1]; // z falcon, y sim
    //    //std::cout << "force exceeded" << std::endl;
    //    //std::cout << -7*pos[0] << std::endl;
    //    //std::cout << force_apply[0] << std::endl;
    //  }
    //}
    //else 
    //{
    //  if (!(abs(force[0])>t_threshold || abs(force[1])>t_threshold || abs(force[2])>t_threshold)) // Torque
    //  {
    //    force_apply[0] = -7*pos[0];
    //    force_apply[1] = -7*pos[1];
    //    force_apply[2] = -7*pos[2];
    //  }
    //  else
    //  {
    //    force_apply[0] = -abs(force[0])*pos[0]; // x falcon, x sim
    //    force_apply[1] = -abs(force[1])*pos[1]; // y falcon, y sim
    //    force_apply[2] = -abs(force[2])*pos[2]; // z falcon, z sim
    //    //std::cout << "torque exceeded" << std::endl;
    //    //std::cout << -7*pos[0] << std::endl;
    //    //std::cout << force_apply[0] << std::endl;
    //  }
    //}
    
    falcon.setForce(force_apply);
    
    //ros::spinOnce(); 
    //loop_rate.sleep();    
  }
  
  falcon.close();

  return 0;
}


