#include "RosInterpolatorFri.h"
#include <getopt.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "lwr_ros2fri");

  std::string rosSetJointTopic = "/robot/lwr/set_joint";
  std::string rosGetJointTopic = "/robot/lwr/get_joint";
  std::string rosSetCartesianTopic = "/robot/lwr/set_cartesian";
  std::string rosGetCartesianTopic = "/robot/lwr/get_cartesian";
  uint16_t friSrcPort = 40008;
  uint16_t friDestPort = 49938;

  const char optstring[] = "";
  struct option longopts[] = {
    { "rossetjointtopic", required_argument, NULL, 0 },
    { "rosgetjointtopic", required_argument, NULL, 0 },
    { "rossetcartesiantopic", required_argument, NULL, 0 },
    { "rosgetcartesiantopic", required_argument, NULL, 0 },
    { "frisrcport", required_argument, NULL, 0 },
    { "fridestport", required_argument, NULL, 0 },
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
      } else if (strcmp(longopts[optindex].name, "rossetcartesiantopic") == 0) {
        rosSetCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "rosgetcartesiantopic") == 0) {
        rosGetCartesianTopic = optarg;
      } else if (strcmp(longopts[optindex].name, "frisrcport") == 0) {
        friSrcPort = atoi(optarg);
      } else if (strcmp(longopts[optindex].name, "fridestport") == 0) {
        friDestPort = atoi(optarg);
      }
      break;
    }
  }

  RosInterpolatorFri rosInterpolatorFri(rosSetJointTopic, rosGetJointTopic, rosSetCartesianTopic, rosGetCartesianTopic, friSrcPort, friDestPort);

  ros::spin();

  return 0;
}