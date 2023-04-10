// Generated by gencpp from file jaka_moveit_action/jakacontrollerFeedback.msg
// DO NOT EDIT!


#ifndef JAKA_MOVEIT_ACTION_MESSAGE_JAKACONTROLLERFEEDBACK_H
#define JAKA_MOVEIT_ACTION_MESSAGE_JAKACONTROLLERFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jaka_moveit_action
{
template <class ContainerAllocator>
struct jakacontrollerFeedback_
{
  typedef jakacontrollerFeedback_<ContainerAllocator> Type;

  jakacontrollerFeedback_()
    : point_num(0)
    , robot_now()  {
    }
  jakacontrollerFeedback_(const ContainerAllocator& _alloc)
    : point_num(0)
    , robot_now(_alloc)  {
  (void)_alloc;
    }



   typedef int16_t _point_num_type;
  _point_num_type point_num;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _robot_now_type;
  _robot_now_type robot_now;





  typedef boost::shared_ptr< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct jakacontrollerFeedback_

typedef ::jaka_moveit_action::jakacontrollerFeedback_<std::allocator<void> > jakacontrollerFeedback;

typedef boost::shared_ptr< ::jaka_moveit_action::jakacontrollerFeedback > jakacontrollerFeedbackPtr;
typedef boost::shared_ptr< ::jaka_moveit_action::jakacontrollerFeedback const> jakacontrollerFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator1> & lhs, const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.point_num == rhs.point_num &&
    lhs.robot_now == rhs.robot_now;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator1> & lhs, const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jaka_moveit_action

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "899863511fa6ada8261fac222bf72461";
  }

  static const char* value(const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x899863511fa6ada8ULL;
  static const uint64_t static_value2 = 0x261fac222bf72461ULL;
};

template<class ContainerAllocator>
struct DataType< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jaka_moveit_action/jakacontrollerFeedback";
  }

  static const char* value(const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define a feedback message\n"
"int16 point_num\n"
"float32[] robot_now\n"
"\n"
;
  }

  static const char* value(const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point_num);
      stream.next(m.robot_now);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct jakacontrollerFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jaka_moveit_action::jakacontrollerFeedback_<ContainerAllocator>& v)
  {
    s << indent << "point_num: ";
    Printer<int16_t>::stream(s, indent + "  ", v.point_num);
    s << indent << "robot_now[]" << std::endl;
    for (size_t i = 0; i < v.robot_now.size(); ++i)
    {
      s << indent << "  robot_now[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.robot_now[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JAKA_MOVEIT_ACTION_MESSAGE_JAKACONTROLLERFEEDBACK_H
