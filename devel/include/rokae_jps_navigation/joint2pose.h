// Generated by gencpp from file rokae_jps_navigation/joint2pose.msg
// DO NOT EDIT!


#ifndef ROKAE_JPS_NAVIGATION_MESSAGE_JOINT2POSE_H
#define ROKAE_JPS_NAVIGATION_MESSAGE_JOINT2POSE_H

#include <ros/service_traits.h>


#include <rokae_jps_navigation/joint2poseRequest.h>
#include <rokae_jps_navigation/joint2poseResponse.h>


namespace rokae_jps_navigation
{

struct joint2pose
{

typedef joint2poseRequest Request;
typedef joint2poseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct joint2pose
} // namespace rokae_jps_navigation


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rokae_jps_navigation::joint2pose > {
  static const char* value()
  {
    return "8e3735eb3bfe1ca9ba91861cd4e5d3f8";
  }

  static const char* value(const ::rokae_jps_navigation::joint2pose&) { return value(); }
};

template<>
struct DataType< ::rokae_jps_navigation::joint2pose > {
  static const char* value()
  {
    return "rokae_jps_navigation/joint2pose";
  }

  static const char* value(const ::rokae_jps_navigation::joint2pose&) { return value(); }
};


// service_traits::MD5Sum< ::rokae_jps_navigation::joint2poseRequest> should match
// service_traits::MD5Sum< ::rokae_jps_navigation::joint2pose >
template<>
struct MD5Sum< ::rokae_jps_navigation::joint2poseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rokae_jps_navigation::joint2pose >::value();
  }
  static const char* value(const ::rokae_jps_navigation::joint2poseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rokae_jps_navigation::joint2poseRequest> should match
// service_traits::DataType< ::rokae_jps_navigation::joint2pose >
template<>
struct DataType< ::rokae_jps_navigation::joint2poseRequest>
{
  static const char* value()
  {
    return DataType< ::rokae_jps_navigation::joint2pose >::value();
  }
  static const char* value(const ::rokae_jps_navigation::joint2poseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rokae_jps_navigation::joint2poseResponse> should match
// service_traits::MD5Sum< ::rokae_jps_navigation::joint2pose >
template<>
struct MD5Sum< ::rokae_jps_navigation::joint2poseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rokae_jps_navigation::joint2pose >::value();
  }
  static const char* value(const ::rokae_jps_navigation::joint2poseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rokae_jps_navigation::joint2poseResponse> should match
// service_traits::DataType< ::rokae_jps_navigation::joint2pose >
template<>
struct DataType< ::rokae_jps_navigation::joint2poseResponse>
{
  static const char* value()
  {
    return DataType< ::rokae_jps_navigation::joint2pose >::value();
  }
  static const char* value(const ::rokae_jps_navigation::joint2poseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROKAE_JPS_NAVIGATION_MESSAGE_JOINT2POSE_H
