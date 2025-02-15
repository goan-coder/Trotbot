// Generated by gencpp from file navigation/PointArray.msg
// DO NOT EDIT!


#ifndef NAVIGATION_MESSAGE_POINTARRAY_H
#define NAVIGATION_MESSAGE_POINTARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <navigation/Point_xy.h>

namespace navigation
{
template <class ContainerAllocator>
struct PointArray_
{
  typedef PointArray_<ContainerAllocator> Type;

  PointArray_()
    : points()  {
    }
  PointArray_(const ContainerAllocator& _alloc)
    : points(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::navigation::Point_xy_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::navigation::Point_xy_<ContainerAllocator> >::other >  _points_type;
  _points_type points;





  typedef boost::shared_ptr< ::navigation::PointArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation::PointArray_<ContainerAllocator> const> ConstPtr;

}; // struct PointArray_

typedef ::navigation::PointArray_<std::allocator<void> > PointArray;

typedef boost::shared_ptr< ::navigation::PointArray > PointArrayPtr;
typedef boost::shared_ptr< ::navigation::PointArray const> PointArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation::PointArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation::PointArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace navigation

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'navigation': ['/home/tyjo/catkin_ws_o/test_codes/Trotbot/trotbot_ws/src/navigation/msg', '/home/tyjo/catkin_ws_o/test_codes/Trotbot/trotbot_ws/src/navigation/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::navigation::PointArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation::PointArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation::PointArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation::PointArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::PointArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::PointArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation::PointArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c2c9f601cd3038d8ee0a839b54636d2";
  }

  static const char* value(const ::navigation::PointArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c2c9f601cd3038dULL;
  static const uint64_t static_value2 = 0x8ee0a839b54636d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation::PointArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation/PointArray";
  }

  static const char* value(const ::navigation::PointArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation::PointArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation/Point_xy[] points\n\
  \n\
================================================================================\n\
MSG: navigation/Point_xy\n\
float32[] point\n\
";
  }

  static const char* value(const ::navigation::PointArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation::PointArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation::PointArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation::PointArray_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::navigation::Point_xy_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION_MESSAGE_POINTARRAY_H
