#ifndef _ROS_INTERPOLATOR_FRI_H_
#define _ROS_INTERPOLATOR_FRI_H_

// system includes

// library includes

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

    // overwritten methods

    // methods

    // variables


  private:
    // methods

    // variables


};

#endif // _ROS_INTERPOLATOR_FRI_H_
