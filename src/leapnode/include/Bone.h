// Generated by gencpp from file leap_motion/Bone.msg
// DO NOT EDIT!


#ifndef LEAP_MOTION_MESSAGE_BONE_H
#define LEAP_MOTION_MESSAGE_BONE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose.h>

namespace leap_motion
{
template <class ContainerAllocator>
struct Bone_
{
  typedef Bone_<ContainerAllocator> Type;

  Bone_()
    : header()
    , type(0)
    , length(0.0)
    , width(0.0)
    , to_string()
    , bone_start()
    , bone_end()
    , center()  {
    }
  Bone_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , type(0)
    , length(0.0)
    , width(0.0)
    , to_string(_alloc)
    , bone_start(_alloc)
    , bone_end(_alloc)
    , center(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _type_type;
  _type_type type;

   typedef float _length_type;
  _length_type length;

   typedef float _width_type;
  _width_type width;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _to_string_type;
  _to_string_type to_string;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _bone_start_type;
  _bone_start_type bone_start;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _bone_end_type;
  _bone_end_type bone_end;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _center_type;
  _center_type center;





  typedef boost::shared_ptr< ::leap_motion::Bone_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::leap_motion::Bone_<ContainerAllocator> const> ConstPtr;

}; // struct Bone_

typedef ::leap_motion::Bone_<std::allocator<void> > Bone;

typedef boost::shared_ptr< ::leap_motion::Bone > BonePtr;
typedef boost::shared_ptr< ::leap_motion::Bone const> BoneConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::leap_motion::Bone_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::leap_motion::Bone_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace leap_motion

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'leap_motion': ['/home/xander/LEAP/src/leap_motion/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::leap_motion::Bone_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::leap_motion::Bone_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::leap_motion::Bone_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::leap_motion::Bone_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leap_motion::Bone_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leap_motion::Bone_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::leap_motion::Bone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ce39afad61bdb16ab802b0100be50795";
  }

  static const char* value(const ::leap_motion::Bone_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xce39afad61bdb16aULL;
  static const uint64_t static_value2 = 0xb802b0100be50795ULL;
};

template<class ContainerAllocator>
struct DataType< ::leap_motion::Bone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "leap_motion/Bone";
  }

  static const char* value(const ::leap_motion::Bone_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::leap_motion::Bone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
\n\
# The name of this bone. \n\
uint8 type\n\
\n\
# The estimated length of the bone in meters.\n\
float32 length\n\
\n\
# The estimated with of the bone in meters.\n\
float32 width\n\
\n\
# A string containing a brief, human readable description of the Bone object. \n\
string to_string\n\
\n\
# The base of the bone, closest to the wrist. \n\
geometry_msgs/Pose bone_start\n\
\n\
# The end of the bone, closest to the finger tip. \n\
geometry_msgs/Pose bone_end\n\
\n\
# The midpoint of the bone. \n\
float32[] center\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::leap_motion::Bone_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::leap_motion::Bone_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.type);
      stream.next(m.length);
      stream.next(m.width);
      stream.next(m.to_string);
      stream.next(m.bone_start);
      stream.next(m.bone_end);
      stream.next(m.center);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Bone_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::leap_motion::Bone_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::leap_motion::Bone_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "length: ";
    Printer<float>::stream(s, indent + "  ", v.length);
    s << indent << "width: ";
    Printer<float>::stream(s, indent + "  ", v.width);
    s << indent << "to_string: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.to_string);
    s << indent << "bone_start: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.bone_start);
    s << indent << "bone_end: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.bone_end);
    s << indent << "center[]" << std::endl;
    for (size_t i = 0; i < v.center.size(); ++i)
    {
      s << indent << "  center[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.center[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LEAP_MOTION_MESSAGE_BONE_H
