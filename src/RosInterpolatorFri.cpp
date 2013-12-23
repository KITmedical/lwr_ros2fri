#include "RosInterpolatorFri.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
RosInterpolatorFri::RosInterpolatorFri(const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosSetCartesianTopic, const std::string& p_rosGetCartesianTopic, uint16_t p_friSrcPort, uint16_t p_friDestPort)
  :m_rosSetJointTopic(p_rosSetJointTopic),
   m_rosGetJointTopic(p_rosGetJointTopic),
   m_rosSetCartesianTopic(p_rosSetCartesianTopic),
   m_rosGetCartesianTopic(p_rosGetCartesianTopic),
   m_friSrcPort(p_friSrcPort),
   m_friDestPort(p_friDestPort)
{
  // TODO next: start FRI Thread
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
