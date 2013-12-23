#include "RosInterpolatorFri.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
RosInterpolatorFri::RosInterpolatorFri(const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosSetCartesianTopic, const std::string& p_rosGetCartesianTopic, uint16_t p_friRecvPort, uint16_t p_friSendPort)
  :m_rosSetJointTopic(p_rosSetJointTopic),
   m_rosGetJointTopic(p_rosGetJointTopic),
   m_rosSetCartesianTopic(p_rosSetCartesianTopic),
   m_rosGetCartesianTopic(p_rosGetCartesianTopic),
   m_friRecvPort(p_friRecvPort),
   m_friSendPort(p_friSendPort)
{
  m_friThread = new boost::thread(boost::bind(&RosInterpolatorFri::runFri, this));
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
RosInterpolatorFri::runFri()
{
  m_friIoService = new boost::asio::io_service();
  m_friRecvSocket = new boost::asio::ip::udp::socket(*m_friIoService, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), m_friRecvPort));

  friRecvStart();

  m_friIoService->run();
}

void
RosInterpolatorFri::friRecvStart()
{
  m_friRecvSocket->async_receive_from(boost::asio::buffer(m_friRecvBuffer),
                                      m_friRecvEndpoint,
                                      boost::bind(&RosInterpolatorFri::friRecvCallback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void
RosInterpolatorFri::friRecvCallback(const boost::system::error_code& p_error, std::size_t p_recvLength)
{
  if (p_error) {
    ROS_FATAL_STREAM("friRecvCallback: " << p_error);
    return;
  }

  if (p_recvLength != sizeof(m_lastFriMsrData)) {
    ROS_FATAL_STREAM("Received size (" << p_recvLength << ") does not match tFriMsrData size (" << sizeof(m_lastFriMsrData) << ")");
    return;
  }

  memcpy(&m_lastFriMsrData, m_friRecvBuffer.data(), sizeof(m_lastFriMsrData));
  printFri(m_lastFriMsrData);

  // TODO send tFriCmdData
 
  friRecvStart();
}

void
RosInterpolatorFri::printFri(const tFriMsrData& p_friMsrData)
{
  printf("head: sendSeqCount=%d reflSeqCount=%d\n", p_friMsrData.head.sendSeqCount, p_friMsrData.head.reflSeqCount);
  printf("intf: timestamp=%lf\n", p_friMsrData.intf.timestamp);
  printf("data:\n");
  printf("  msrJntPos: ");
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    printf("%1.3lf ", p_friMsrData.data.msrJntPos[jointIdx]);
  }
  printf("\n");
  printf("---\n");
}
/*------------------------------------------------------------------------}}}-*/
