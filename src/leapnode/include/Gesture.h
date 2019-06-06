// Generated by gencpp from file leap_motion/Gesture.msg
// DO NOT EDIT!


#ifndef LEAP_MOTION_MESSAGE_GESTURE_H
#define LEAP_MOTION_MESSAGE_GESTURE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace leap_motion
{
template <class ContainerAllocator>
struct Gesture_
{
  typedef Gesture_<ContainerAllocator> Type;

  Gesture_()
    : lmc_gesture_id(0)
    , is_valid(false)
    , duration_us(0)
    , duration_s(0.0)
    , gesture_state(0)
    , gesture_type(0)
    , to_string()
    , pointable_ids()  {
    }
  Gesture_(const ContainerAllocator& _alloc)
    : lmc_gesture_id(0)
    , is_valid(false)
    , duration_us(0)
    , duration_s(0.0)
    , gesture_state(0)
    , gesture_type(0)
    , to_string(_alloc)
    , pointable_ids(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _lmc_gesture_id_type;
  _lmc_gesture_id_type lmc_gesture_id;

   typedef uint8_t _is_valid_type;
  _is_valid_type is_valid;

   typedef int64_t _duration_us_type;
  _duration_us_type duration_us;

   typedef float _duration_s_type;
  _duration_s_type duration_s;

   typedef int32_t _gesture_state_type;
  _gesture_state_type gesture_state;

   typedef int32_t _gesture_type_type;
  _gesture_type_type gesture_type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _to_string_type;
  _to_string_type to_string;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _pointable_ids_type;
  _pointable_ids_type pointable_ids;





  typedef boost::shared_ptr< ::leap_motion::Gesture_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::leap_motion::Gesture_<ContainerAllocator> const> ConstPtr;

}; // struct Gesture_

typedef ::leap_motion::Gesture_<std::allocator<void> > Gesture;

typedef boost::shared_ptr< ::leap_motion::Gesture > GesturePtr;
typedef boost::shared_ptr< ::leap_motion::Gesture const> GestureConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::leap_motion::Gesture_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::leap_motion::Gesture_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace leap_motion

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'leap_motion': ['/home/xander/LEAP/src/leap_motion/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::leap_motion::Gesture_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::leap_motion::Gesture_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::leap_motion::Gesture_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::leap_motion::Gesture_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leap_motion::Gesture_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leap_motion::Gesture_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::leap_motion::Gesture_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a3bf4fe2d0e707244a5a679250ea8eba";
  }

  static const char* value(const ::leap_motion::Gesture_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa3bf4fe2d0e70724ULL;
  static const uint64_t static_value2 = 0x4a5a679250ea8ebaULL;
};

template<class ContainerAllocator>
struct DataType< ::leap_motion::Gesture_<ContainerAllocator> >
{
  static const char* value()
  {
    return "leap_motion/Gesture";
  }

  static const char* value(const ::leap_motion::Gesture_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::leap_motion::Gesture_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# A unique id given to this gesture\n\
int32 lmc_gesture_id\n\
\n\
# Reports whether this Gesture instance represents a valid Gesture. \n\
bool is_valid\n\
\n\
# The elapsed duration of the recognized movement up to the frame containing this \n\
# Gesture object, in microseconds. \n\
int64 duration_us\n\
\n\
# The elapsed duration in seconds.\n\
float32 duration_s\n\
\n\
# Recognized movements occur over time and have a beginning, a middle, and an end. \n\
# The 'state' attribute reports where in that sequence this Gesture object falls.\n\
int32 gesture_state\n\
### STATE_INVALID == -1\n\
### STATE_START == 1\n\
### STATE_UPDATE == 2\n\
### STATE_STOP == 3\n\
\n\
# The supported types of gestures. \n\
int32 gesture_type\n\
### TYPE_INVALID == -1\n\
### TYPE_SWIPE == 1\n\
### TYPE_CIRCLE == 4\n\
### TYPE_SCREEN_TAP == 5\n\
### TYPE_KEY_TAP == 6\n\
\n\
# A string containing a brief, human-readable description of this Gesture. \n\
string to_string\n\
\n\
# The list of fingers, tools ids associated with this Gesture, if any. \n\
int32[] pointable_ids\n\
\n\
";
  }

  static const char* value(const ::leap_motion::Gesture_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::leap_motion::Gesture_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lmc_gesture_id);
      stream.next(m.is_valid);
      stream.next(m.duration_us);
      stream.next(m.duration_s);
      stream.next(m.gesture_state);
      stream.next(m.gesture_type);
      stream.next(m.to_string);
      stream.next(m.pointable_ids);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Gesture_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::leap_motion::Gesture_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::leap_motion::Gesture_<ContainerAllocator>& v)
  {
    s << indent << "lmc_gesture_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lmc_gesture_id);
    s << indent << "is_valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_valid);
    s << indent << "duration_us: ";
    Printer<int64_t>::stream(s, indent + "  ", v.duration_us);
    s << indent << "duration_s: ";
    Printer<float>::stream(s, indent + "  ", v.duration_s);
    s << indent << "gesture_state: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gesture_state);
    s << indent << "gesture_type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gesture_type);
    s << indent << "to_string: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.to_string);
    s << indent << "pointable_ids[]" << std::endl;
    for (size_t i = 0; i < v.pointable_ids.size(); ++i)
    {
      s << indent << "  pointable_ids[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.pointable_ids[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LEAP_MOTION_MESSAGE_GESTURE_H
