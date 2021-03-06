// Generated by gencpp from file rokae_jps_navigation/GotoResponse.msg
// DO NOT EDIT!


#ifndef ROKAE_JPS_NAVIGATION_MESSAGE_GOTORESPONSE_H
#define ROKAE_JPS_NAVIGATION_MESSAGE_GOTORESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rokae_jps_navigation
{
template <class ContainerAllocator>
struct GotoResponse_
{
  typedef GotoResponse_<ContainerAllocator> Type;

  GotoResponse_()
    : message()
    , success(false)
    , px()
    , py()
    , pz()
    , pos()
    , vel()
    , acc()
    , t()
    , back_pos()
    , back_vel()
    , back_acc()
    , back_t()  {
    }
  GotoResponse_(const ContainerAllocator& _alloc)
    : message(_alloc)
    , success(false)
    , px(_alloc)
    , py(_alloc)
    , pz(_alloc)
    , pos(_alloc)
    , vel(_alloc)
    , acc(_alloc)
    , t(_alloc)
    , back_pos(_alloc)
    , back_vel(_alloc)
    , back_acc(_alloc)
    , back_t(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;

   typedef uint8_t _success_type;
  _success_type success;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _px_type;
  _px_type px;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _py_type;
  _py_type py;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _pz_type;
  _pz_type pz;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _pos_type;
  _pos_type pos;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _vel_type;
  _vel_type vel;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _acc_type;
  _acc_type acc;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _t_type;
  _t_type t;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _back_pos_type;
  _back_pos_type back_pos;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _back_vel_type;
  _back_vel_type back_vel;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _back_acc_type;
  _back_acc_type back_acc;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _back_t_type;
  _back_t_type back_t;





  typedef boost::shared_ptr< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GotoResponse_

typedef ::rokae_jps_navigation::GotoResponse_<std::allocator<void> > GotoResponse;

typedef boost::shared_ptr< ::rokae_jps_navigation::GotoResponse > GotoResponsePtr;
typedef boost::shared_ptr< ::rokae_jps_navigation::GotoResponse const> GotoResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator1> & lhs, const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator2> & rhs)
{
  return lhs.message == rhs.message &&
    lhs.success == rhs.success &&
    lhs.px == rhs.px &&
    lhs.py == rhs.py &&
    lhs.pz == rhs.pz &&
    lhs.pos == rhs.pos &&
    lhs.vel == rhs.vel &&
    lhs.acc == rhs.acc &&
    lhs.t == rhs.t &&
    lhs.back_pos == rhs.back_pos &&
    lhs.back_vel == rhs.back_vel &&
    lhs.back_acc == rhs.back_acc &&
    lhs.back_t == rhs.back_t;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator1> & lhs, const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rokae_jps_navigation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d1f4162acc575e9ecdf795508e248a9";
  }

  static const char* value(const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d1f4162acc575e9ULL;
  static const uint64_t static_value2 = 0xecdf795508e248a9ULL;
};

template<class ContainerAllocator>
struct DataType< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rokae_jps_navigation/GotoResponse";
  }

  static const char* value(const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string message\n"
"bool success\n"
"float64[] px\n"
"float64[] py\n"
"float64[] pz\n"
"float64[] pos\n"
"float64[] vel\n"
"float64[] acc\n"
"float64[] t\n"
"float64[] back_pos\n"
"float64[] back_vel\n"
"float64[] back_acc\n"
"float64[] back_t\n"
;
  }

  static const char* value(const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.message);
      stream.next(m.success);
      stream.next(m.px);
      stream.next(m.py);
      stream.next(m.pz);
      stream.next(m.pos);
      stream.next(m.vel);
      stream.next(m.acc);
      stream.next(m.t);
      stream.next(m.back_pos);
      stream.next(m.back_vel);
      stream.next(m.back_acc);
      stream.next(m.back_t);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GotoResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rokae_jps_navigation::GotoResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rokae_jps_navigation::GotoResponse_<ContainerAllocator>& v)
  {
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "px[]" << std::endl;
    for (size_t i = 0; i < v.px.size(); ++i)
    {
      s << indent << "  px[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.px[i]);
    }
    s << indent << "py[]" << std::endl;
    for (size_t i = 0; i < v.py.size(); ++i)
    {
      s << indent << "  py[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.py[i]);
    }
    s << indent << "pz[]" << std::endl;
    for (size_t i = 0; i < v.pz.size(); ++i)
    {
      s << indent << "  pz[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.pz[i]);
    }
    s << indent << "pos[]" << std::endl;
    for (size_t i = 0; i < v.pos.size(); ++i)
    {
      s << indent << "  pos[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.pos[i]);
    }
    s << indent << "vel[]" << std::endl;
    for (size_t i = 0; i < v.vel.size(); ++i)
    {
      s << indent << "  vel[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.vel[i]);
    }
    s << indent << "acc[]" << std::endl;
    for (size_t i = 0; i < v.acc.size(); ++i)
    {
      s << indent << "  acc[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.acc[i]);
    }
    s << indent << "t[]" << std::endl;
    for (size_t i = 0; i < v.t.size(); ++i)
    {
      s << indent << "  t[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.t[i]);
    }
    s << indent << "back_pos[]" << std::endl;
    for (size_t i = 0; i < v.back_pos.size(); ++i)
    {
      s << indent << "  back_pos[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.back_pos[i]);
    }
    s << indent << "back_vel[]" << std::endl;
    for (size_t i = 0; i < v.back_vel.size(); ++i)
    {
      s << indent << "  back_vel[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.back_vel[i]);
    }
    s << indent << "back_acc[]" << std::endl;
    for (size_t i = 0; i < v.back_acc.size(); ++i)
    {
      s << indent << "  back_acc[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.back_acc[i]);
    }
    s << indent << "back_t[]" << std::endl;
    for (size_t i = 0; i < v.back_t.size(); ++i)
    {
      s << indent << "  back_t[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.back_t[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROKAE_JPS_NAVIGATION_MESSAGE_GOTORESPONSE_H
