#include "RosInterpolatorFri.h"

// system includes

// library includes

// custom includes
#include <ahbstring.h>
#include <lwr/LwrLibrary.hpp>


/*---------------------------------- public: -----------------------------{{{-*/
RosInterpolatorFri::RosInterpolatorFri(const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosStateTopic, uint16_t p_friRecvPort, uint16_t p_friSendPort)
  :m_rosSetJointTopic(p_rosSetJointTopic),
   m_rosGetJointTopic(p_rosGetJointTopic),
   m_rosStateTopic(p_rosStateTopic),
   m_friRecvPort(p_friRecvPort),
   m_friSendPort(p_friSendPort),
   m_gpi(LBR_MNJ),
   m_gpiPosCurrentBuffer(LBR_MNJ, 0),
   m_gpiPosTargetBuffer(LBR_MNJ, 0),
   m_gpiPosMinBuffer(LBR_MNJ, 0),
   m_gpiPosMaxBuffer(LBR_MNJ, 0),
   m_gpiVelCurrentBuffer(LBR_MNJ, 0),
   m_gpiVelMaxBuffer(LBR_MNJ, 0.4),
   m_gpiAccelMaxBuffer(LBR_MNJ, 0.4)
{
  // gpi
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    m_gpiPosMinBuffer[jointIdx] = -1 * Lwr::jointLimits.j[jointIdx];
    m_gpiPosMaxBuffer[jointIdx] = Lwr::jointLimits.j[jointIdx];
  }
  m_gpi.setXTarget(m_gpiPosTargetBuffer);
  m_gpi.setXLast(m_gpiPosCurrentBuffer);
  m_gpi.setVLast(m_gpiVelCurrentBuffer);
  m_gpi.setXMin(m_gpiPosMinBuffer);
  m_gpi.setXMax(m_gpiPosMaxBuffer);
  m_gpi.setVMax(m_gpiVelMaxBuffer);
  m_gpi.setAMax(m_gpiAccelMaxBuffer);
  m_gpi.setDt(0.001);
  m_gpi.setMode(1);

  // fri
  memset(&m_currentFriCmdData, 0, sizeof(m_currentFriCmdData));
  m_currentFriCmdData.head.packetSize = FRI_CMD_DATA_SIZE;
  m_currentFriCmdData.head.datagramId = FRI_DATAGRAM_ID_CMD;
  m_currentFriCmdData.cmd.cmdFlags = FRI_CMD_JNTPOS;

  m_friThread = new boost::thread(boost::bind(&RosInterpolatorFri::runFri, this));

  // ros
  m_rosCurrentJointState.position.resize(LBR_MNJ, 0);
  m_rosCurrentJointState.velocity.resize(LBR_MNJ, 0);
  m_rosCurrentJointState.effort.resize(LBR_MNJ, 0);

  m_rosSetJointTopicSub = m_rosNode.subscribe<sensor_msgs::JointState>(m_rosSetJointTopic, 1, &RosInterpolatorFri::rosSetJointCallback, this);
  m_rosGetJointTopicPub = m_rosNode.advertise<sensor_msgs::JointState>(m_rosGetJointTopic, 1);
  m_rosStateTopicPub = m_rosNode.advertise<std_msgs::String>(m_rosStateTopic, 1);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
RosInterpolatorFri::runFri()
{
  m_friIoService = new boost::asio::io_service();
  m_friRecvSocket = new boost::asio::ip::udp::socket(*m_friIoService, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), m_friRecvPort));

  m_friSendSocket = new boost::asio::ip::udp::socket(*m_friIoService);
  m_friResolver = new boost::asio::ip::udp::resolver(*m_friIoService);
  boost::asio::ip::udp::resolver::query sendPortQuery(boost::asio::ip::udp::v4(),
                                                      "localhost",
                                                      ahb::string::toString(m_friSendPort));
  m_friSendEndpoint = *(m_friResolver->resolve(sendPortQuery));

  try {
    m_friSendSocket->connect(m_friSendEndpoint);
  } catch (boost::system::system_error const& e) {
    ROS_FATAL_STREAM("RosInterpolatorFri: Failed to connect to FRI robot: " << e.what());
  }

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

  updateRosFromFri();
  rosPublish();

  m_gpi.interpolate();
  m_gpi.getXNow(m_gpiPosCurrentBuffer);
  m_gpi.getVNow(m_gpiVelCurrentBuffer);
  std::cout << "m_gpiPosCurrentBuffer: " << ahb::string::toString(m_gpiPosCurrentBuffer) << std::endl;

  m_currentFriCmdData.head.sendSeqCount++;
  m_currentFriCmdData.head.reflSeqCount = m_lastFriMsrData.head.sendSeqCount;
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    m_currentFriCmdData.cmd.jntPos[jointIdx] = m_gpiPosCurrentBuffer[jointIdx];
  }
  try {
    m_friSendSocket->send(boost::asio::buffer(&m_currentFriCmdData, sizeof(m_currentFriCmdData)));
  } catch (boost::system::system_error const& e) {
    ROS_FATAL_STREAM("RosInterpolatorFri: Failed to send to FRI robot: " << e.what());
  }
 
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

void
RosInterpolatorFri::updateRosFromFri()
{
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    m_rosCurrentJointState.position[jointIdx] = m_lastFriMsrData.data.msrJntPos[jointIdx];
  }
}


void
RosInterpolatorFri::rosSetJointCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
{
  std::cout << "rosSetJointCallback: jointsMsg=" << *jointsMsg << std::endl;
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    m_gpiPosTargetBuffer[jointIdx] = jointsMsg->position[jointIdx];
  }
  m_gpi.setXTarget(m_gpiPosTargetBuffer);
}

void
RosInterpolatorFri::rosPublish()
{
  m_rosGetJointTopicPub.publish(m_rosCurrentJointState);
}
/*------------------------------------------------------------------------}}}-*/
