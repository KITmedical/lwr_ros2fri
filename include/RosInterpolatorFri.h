#ifndef _ROS_INTERPOLATOR_FRI_H_
#define _ROS_INTERPOLATOR_FRI_H_

// system includes

// library includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <friComm.h>

// custom includes
#include <gpi/GeneralPurposeInterpolator.hpp>

// forward declarations


class RosInterpolatorFri
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    RosInterpolatorFri(const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosStateTopic, uint16_t p_friRecvPort, uint16_t p_friSendPort);

    // overwritten methods

    // methods

    // variables


  private:
    // methods
    void runFri();
    void friRecvStart();
    void friRecvCallback(const boost::system::error_code& p_error, std::size_t p_recvLength);
    void printFri(const tFriMsrData& p_friMsrData);

    void updateRosFromFri();

    void rosSetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg);
    void rosPublish();

    // variables
    std::string m_rosSetJointTopic;
    std::string m_rosGetJointTopic;
    std::string m_rosStateTopic;

    uint16_t m_friRecvPort;
    uint16_t m_friSendPort;
    boost::thread* m_friThread;
    boost::asio::io_service* m_friIoService;
    boost::asio::ip::udp::socket* m_friRecvSocket;
    boost::asio::ip::udp::endpoint m_friRecvEndpoint;
    boost::asio::ip::udp::resolver* m_friResolver;
    boost::asio::ip::udp::socket* m_friSendSocket;
    boost::asio::ip::udp::endpoint m_friSendEndpoint;
    boost::array<char, sizeof(tFriMsrData)> m_friRecvBuffer;
    tFriMsrData m_lastFriMsrData;
    tFriCmdData m_currentFriCmdData;

    ros::NodeHandle m_rosNode;
    ros::Subscriber m_rosSetJointTopicSub;
    ros::Publisher m_rosGetJointTopicPub;
    ros::Publisher m_rosStateTopicPub;
    sensor_msgs::JointState m_rosCurrentJointState;

    GeneralPurposeInterpolator m_gpi;
    std::vector<double> m_gpiPosCurrentBuffer;
    std::vector<double> m_gpiPosTargetBuffer;
    std::vector<double> m_gpiPosMinBuffer;
    std::vector<double> m_gpiPosMaxBuffer;
    std::vector<double> m_gpiVelCurrentBuffer;
    std::vector<double> m_gpiVelMaxBuffer;
    std::vector<double> m_gpiAccelMaxBuffer;
};

#endif // _ROS_INTERPOLATOR_FRI_H_
