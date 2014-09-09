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
#include <GeneralPurposeInterpolator.hpp>

// forward declarations


class RosInterpolatorFri
{
  public:
    // enums

    // typedefs

    // const static member variables
    const static std::size_t m_friThreadsCount = 3;
    const static double m_velMax = 0.3;
    const static double m_accelMax = 10.0;
 
    // static utility functions


    // constructors
    RosInterpolatorFri(const std::string& p_robotName, const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosStateTopic, const std::string& p_friHost, uint16_t p_friRecvPort, uint16_t p_friSendPort);

    // overwritten methods

    // methods

    // variables


  private:
    // methods
    void runFri();
    void friRecvStart();
    void friRecvCallback(const boost::system::error_code& p_error, std::size_t p_recvLength);
    void printFri(const tFriMsrData& p_friMsrData);
    void printFriCmd(const tFriCmdData& p_friCmdData);

    void updateRosFromFri();

    void rosSetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg);
    void rosPublish();
    void rosPublishLoop();

    // variables
    std::string m_robotName;
    std::string m_rosSetJointTopic;
    std::string m_rosGetJointTopic;
    std::string m_rosStateTopic;

    std::string m_friHost;
    uint16_t m_friRecvPort;
    uint16_t m_friSendPort;
    std::vector<boost::shared_ptr<boost::thread> > m_friThreads;
    boost::asio::io_service* m_friIoService;
    boost::asio::ip::udp::socket* m_friSocket;
    boost::asio::ip::udp::endpoint m_friRecvEndpoint;
    boost::asio::ip::udp::resolver* m_friResolver;
    boost::asio::ip::udp::endpoint m_friSendEndpoint;
    boost::array<char, sizeof(tFriMsrData)> m_friRecvBuffer;
    tFriMsrData m_lastFriMsrData;
    tFriCmdData m_currentFriCmdData;

    ros::NodeHandle m_rosNode;
    ros::Subscriber m_rosSetJointTopicSub;
    ros::Publisher m_rosGetJointTopicPub;
    ros::Publisher m_rosStateTopicPub;
    std_msgs::String m_currentState;
    sensor_msgs::JointState m_rosCurrentJointState;
    boost::shared_ptr<boost::thread> m_rosPublishThread;
    boost::condition_variable m_rosUpdateCondVar;
    boost::mutex m_rosUpdateMutex;

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
