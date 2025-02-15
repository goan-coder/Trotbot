// Generated by gencpp from file navigation/Point_xy.msg
// DO NOT EDIT!


#ifndef NAVIGATION_MESSAGE_POINT_XY_H
#define NAVIGATION_MESSAGE_POINT_XY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace navigation
{
template <class ContainerAllocator>
struct Point_xy_
{
  typedef Point_xy_<ContainerAllocator> Type;

  Point_xy_()
    : point()  {
    }
  Point_xy_(const ContainerAllocator& _alloc)
    : point(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _point_type;
  _point_type point;





  typedef boost::shared_ptr< ::navigation::Point_xy_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation::Point_xy_<ContainerAllocator> const> ConstPtr;

}; // struct Point_xy_

typedef ::navigation::Point_xy_<std::allocator<void> > Point_xy;

typedef boost::shared_ptr< ::navigation::Point_xy > Point_xyPtr;
typedef boost::shared_ptr< ::navigation::Point_xy const> Point_xyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation::Point_xy_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation::Point_xy_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::navigation::Point_xy_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation::Point_xy_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation::Point_xy_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation::Point_xy_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::Point_xy_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::Point_xy_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation::Point_xy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "318ea976b093c91a3f95a8e83351f8ad";
  }

  static const char* value(const ::navigation::Point_xy_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x318ea976b093c91aULL;
  static const uint64_t static_value2 = 0x3f95a8e83351f8adULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation::Point_xy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation/Point_xy";
  }

  static const char* value(const ::navigation::Point_xy_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation::Point_xy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] point\n\
";
  }

  static const char* value(const ::navigation::Point_xy_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation::Point_xy_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Point_xy_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation::Point_xy_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation::Point_xy_<ContainerAllocator>& v)
  {
    s << indent << "point[]" << std::endl;
    for (size_t i = 0; i < v.point.size(); ++i)
    {
      s << indent << "  point[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.point[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION_MESSAGE_POINT_XY_H
