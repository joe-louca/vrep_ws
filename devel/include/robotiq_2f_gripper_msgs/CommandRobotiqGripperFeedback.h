// Generated by gencpp from file robotiq_2f_gripper_msgs/CommandRobotiqGripperFeedback.msg
// DO NOT EDIT!


#ifndef ROBOTIQ_2F_GRIPPER_MSGS_MESSAGE_COMMANDROBOTIQGRIPPERFEEDBACK_H
#define ROBOTIQ_2F_GRIPPER_MSGS_MESSAGE_COMMANDROBOTIQGRIPPERFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace robotiq_2f_gripper_msgs
{
template <class ContainerAllocator>
struct CommandRobotiqGripperFeedback_
{
  typedef CommandRobotiqGripperFeedback_<ContainerAllocator> Type;

  CommandRobotiqGripperFeedback_()
    : header()
    , is_ready(false)
    , is_reset(false)
    , is_moving(false)
    , obj_detected(false)
    , fault_status(0)
    , position(0.0)
    , requested_position(0.0)
    , current(0.0)  {
    }
  CommandRobotiqGripperFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , is_ready(false)
    , is_reset(false)
    , is_moving(false)
    , obj_detected(false)
    , fault_status(0)
    , position(0.0)
    , requested_position(0.0)
    , current(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _is_ready_type;
  _is_ready_type is_ready;

   typedef uint8_t _is_reset_type;
  _is_reset_type is_reset;

   typedef uint8_t _is_moving_type;
  _is_moving_type is_moving;

   typedef uint8_t _obj_detected_type;
  _obj_detected_type obj_detected;

   typedef uint8_t _fault_status_type;
  _fault_status_type fault_status;

   typedef float _position_type;
  _position_type position;

   typedef float _requested_position_type;
  _requested_position_type requested_position;

   typedef float _current_type;
  _current_type current;





  typedef boost::shared_ptr< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct CommandRobotiqGripperFeedback_

typedef ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<std::allocator<void> > CommandRobotiqGripperFeedback;

typedef boost::shared_ptr< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback > CommandRobotiqGripperFeedbackPtr;
typedef boost::shared_ptr< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback const> CommandRobotiqGripperFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator1> & lhs, const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.is_ready == rhs.is_ready &&
    lhs.is_reset == rhs.is_reset &&
    lhs.is_moving == rhs.is_moving &&
    lhs.obj_detected == rhs.obj_detected &&
    lhs.fault_status == rhs.fault_status &&
    lhs.position == rhs.position &&
    lhs.requested_position == rhs.requested_position &&
    lhs.current == rhs.current;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator1> & lhs, const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotiq_2f_gripper_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b008656b72baf8ed957040c01e114fed";
  }

  static const char* value(const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb008656b72baf8edULL;
  static const uint64_t static_value2 = 0x957040c01e114fedULL;
};

template<class ContainerAllocator>
struct DataType< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotiq_2f_gripper_msgs/CommandRobotiqGripperFeedback";
  }

  static const char* value(const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#feedback\n"
"Header header\n"
"bool is_ready\n"
"bool is_reset\n"
"bool is_moving\n"
"bool obj_detected\n"
"uint8 fault_status\n"
"float32 position\n"
"float32 requested_position\n"
"float32 current\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.is_ready);
      stream.next(m.is_reset);
      stream.next(m.is_moving);
      stream.next(m.obj_detected);
      stream.next(m.fault_status);
      stream.next(m.position);
      stream.next(m.requested_position);
      stream.next(m.current);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommandRobotiqGripperFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "is_ready: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_ready);
    s << indent << "is_reset: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_reset);
    s << indent << "is_moving: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_moving);
    s << indent << "obj_detected: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.obj_detected);
    s << indent << "fault_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fault_status);
    s << indent << "position: ";
    Printer<float>::stream(s, indent + "  ", v.position);
    s << indent << "requested_position: ";
    Printer<float>::stream(s, indent + "  ", v.requested_position);
    s << indent << "current: ";
    Printer<float>::stream(s, indent + "  ", v.current);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIQ_2F_GRIPPER_MSGS_MESSAGE_COMMANDROBOTIQGRIPPERFEEDBACK_H
