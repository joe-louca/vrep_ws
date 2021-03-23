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
std::array<double, 3> pos, oldpos, falcon_force;
std::array<double, 6> force;
double fx, fy, fz, tx, ty, tz, theta, phi, force_mag;
int btn, rate_hz;
float f_threshold, t_threshold;


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


int main(int argc, char* argv[])
{
  std::array<double, 3> move, ft_sensor, F;
  bool rpy_mode;
  string gripper_control;
  
  ros::init(argc, argv, "falcon_stream");
  ros::NodeHandle n;
  ros::Subscriber ft_sub;

  twist_pub = n.advertise<geometry_msgs::TwistStamped>("falcon/twist", 1);

  if(!initialise())
    {return 0;}
    
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
  n.getParam("f_threshold",f_threshold);
  n.getParam("t_threshold",t_threshold);
 
  // Set loop rate
  n.getParam("rate_hz",rate_hz);
  ros::Rate loop_rate(rate_hz);

  oldpos[0] = oldpos[1] = oldpos[2] = 0.0;
  
  while(ros::ok())
  {
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
    
    // If PLUS button pressed, switch to rpy mode
    btn = falcon.getFalconGrip()->getDigitalInputs(); 
    if (btn & libnifalcon::FalconGripFourButton::PLUS_BUTTON)
    {
      rpy_mode = true;
    }
    else 
    {
      rpy_mode = false;
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
    
    // if centre button pressed then allow moving
    if (btn & libnifalcon::FalconGripFourButton::CENTER_BUTTON)
    {    
      // Publish as twist topic
      publish_twist(0.15, pos, rpy_mode, gripper_control); 
      
      // Get ft reading
      n.getParam("ft_delay/fx",fx);
      n.getParam("ft_delay/fy",fy);
      n.getParam("ft_delay/fz",fz);
      n.getParam("ft_delay/tx",tx);
      n.getParam("ft_delay/ty",ty);
      n.getParam("ft_delay/tz",tz);
      ft_sensor = {fx, fz, fy};
      
      // Set stiffness values 
      double stiffness = 4.0;
      double friction = 0.00;
      // TO DO... Could also adjust stiffness depending on rigidity of object contacted 

      // Set threshold values
      f_threshold = 100.0;
      double friction_threshold = 30.0;      
      
      // Set 0 force to start
      F = {0.0, 0.0, 0.0};
      falcon_force = {0.0, 0.0, 0.0};
      
      // Right/Left = +/-[0]
      // Up/Down = +/- [1]
      // Out/in = +/-[2]

      // For each axis, check if force is large - i.e. rigid body contact
      for (int axis = 0; axis < 3; ++axis)
      {
        if (ft_sensor[axis] > f_threshold)
          {F[axis] = ft_sensor[axis];}
      }
      
      // Normalise F to ratios
      double sumF = F[0] + F[1] + F[2];
      F = {F[0]/sumF, F[1]/sumF, F[2]/sumF};
      
      // check which side falcon pos is for the plane normal to F which intersects {0,0,0} (dot product of F (assume normal to surface) and position)
      double dot_prod = F[0]*pos[0] + F[1]*pos[1] + F[2]*pos[2];
      if (dot_prod < 0.0)		// pos is opposite direction of normal force, so apply immovable force in normal direction
        {falcon_force = {stiffness*F[0], stiffness*F[1], stiffness*F[2]};} 
      
      
      // For each axis, check if force is medium - i.e. frictional
      for (int axis = 0; axis < 3; ++axis)
      {
        if ((ft_sensor[axis] > friction_threshold) && (ft_sensor[axis] < f_threshold))
          {F[axis] = ft_sensor[axis];}
        else
          {F[axis] = 0.0;}
      }
      
      // check which side falcon pos is for the plane normal to F which intersects {0,0,0} (dot product of F (assume major friction direction) and position)
      dot_prod = F[0]*pos[0] + F[1]*pos[1] + F[2]*pos[2];
      
      if (dot_prod < 0.0) // pos is opposite direction of friction force, so add a drag force in normal direction
      {
        falcon_force[0] += friction*F[0];
        falcon_force[1] += friction*F[1];
        falcon_force[2] += friction*F[2];
      }
        
      // Set falcon force
      falcon.setForce(falcon_force);           
    }
        
    else
    {
      // Publish no move
      pos = {0.0, 0.0, 0.0};
      publish_twist(0.15, pos, rpy_mode, gripper_control);
      
      // Give no force feedback
      falcon_force = {0.0, 0.0, 0.0};
      falcon.setForce(falcon_force);
    }

    loop_rate.sleep();    
  }
  
  falcon.close();
  return 0;
}


