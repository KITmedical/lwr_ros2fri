#ifndef _ROS_INTERPOLATOR_FRI_H_
#define _ROS_INTERPOLATOR_FRI_H_

// system includes

// library includes
#include <ros/ros.h>

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
    RosInterpolatorFri(const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosSetCartesianTopic, const std::string& p_rosGetCartesianTopic, uint16_t p_friSrcPort, uint16_t p_friDestPort);

    // overwritten methods

    // methods

    // variables


  private:
    // methods

    // variables
    std::string m_rosSetJointTopic;
    std::string m_rosGetJointTopic;
    std::string m_rosSetCartesianTopic;
    std::string m_rosGetCartesianTopic;
    uint16_t m_friSrcPort;
    uint16_t m_friDestPort;


};

#endif // _ROS_INTERPOLATOR_FRI_H_
