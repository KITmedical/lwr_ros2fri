#ifndef _ROS_INTERPOLATOR_FRI_H_
#define _ROS_INTERPOLATOR_FRI_H_

// system includes

// library includes
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <friComm.h>

// custom includes


// forward declarations

/*
 * TODO:
 * - take code from LwrModelPlugin to publish/subscribe ROS topics for robot
 * - use gpi to interpolate joints
 * - look at ICRACK-component.* for conversion to FRI messages (plwr->)
 *
 * Control flow:
 * - ROS Thread: Read incoming, update targets, read fri state, publish to topics
 * - FRI Thread: Wait for incoming fri message, update fri state, do gpi step, send values
 * mutex?
 */

class RosInterpolatorFri
{
  public:
    // enums

    // typedefs

    // const static member variables
 
    // static utility functions


    // constructors
    RosInterpolatorFri(const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosSetCartesianTopic, const std::string& p_rosGetCartesianTopic, uint16_t p_friRecvPort, uint16_t p_friSendPort);

    // overwritten methods

    // methods

    // variables


  private:
    // methods
    void runFri();
    void friRecvStart();
    void friRecvCallback(const boost::system::error_code& p_error, std::size_t p_recvLength);
    void printFri(const tFriMsrData& p_friMsrData);

    // variables
    std::string m_rosSetJointTopic;
    std::string m_rosGetJointTopic;
    std::string m_rosSetCartesianTopic;
    std::string m_rosGetCartesianTopic;

    uint16_t m_friRecvPort;
    uint16_t m_friSendPort;
    boost::thread* m_friThread;
    boost::asio::io_service* m_friIoService;
    boost::asio::ip::udp::socket* m_friRecvSocket;
    boost::asio::ip::udp::endpoint m_friRecvEndpoint;
    boost::array<char, sizeof(tFriMsrData)> m_friRecvBuffer;
    tFriMsrData m_lastFriMsrData;


};

#endif // _ROS_INTERPOLATOR_FRI_H_
