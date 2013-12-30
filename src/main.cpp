#include "RosInterpolatorFri.h"
#include <getopt.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "lwr_ros2fri");

  std::string rosSetJointTopic = "/robots/lwr/direct/set_joint";
  std::string rosGetJointTopic = "/robots/lwr/direct/get_joint";
  uint16_t friRecvPort = 40008;
  uint16_t friSendPort = 49938;

  const char optstring[] = "";
  struct option longopts[] = {
    { "rossetjointtopic", required_argument, NULL, 0 },
    { "rosgetjointtopic", required_argument, NULL, 0 },
    { "frirecvport", required_argument, NULL, 0 },
    { "frisendport", required_argument, NULL, 0 },
  };
  int opt;
  int optindex;
  while ((opt = getopt_long(argc, argv, optstring, longopts, &optindex)) != -1) {
    switch (opt) {
    case 0:
      if (strcmp(longopts[optindex].name, "rossetjointtopic") == 0) {
        rosSetJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "rosgetjointtopic") == 0) {
        rosGetJointTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "frirecvport") == 0) {
        friRecvPort = atoi(optarg);
      } else if (strcmp(longopts[optindex].name, "frisendport") == 0) {
        friSendPort = atoi(optarg);
      }
      break;
    }
  }

  RosInterpolatorFri rosInterpolatorFri(rosSetJointTopic, rosGetJointTopic, friRecvPort, friSendPort);

  std::cout << "Spinning" << std::endl;
  ros::spin();

  return 0;
}
