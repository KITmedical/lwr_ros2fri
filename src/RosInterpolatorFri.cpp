#include "RosInterpolatorFri.h"

// system includes
#include <pthread.h>
#include <sched.h>

// library includes

// custom includes
#include <ahbstring.h>
#include <LwrLibrary.hpp>


/*---------------------------------- public: -----------------------------{{{-*/
RosInterpolatorFri::RosInterpolatorFri(const std::string& p_robotName, const std::string& p_rosSetJointTopic, const std::string& p_rosGetJointTopic, const std::string& p_rosStateTopic, const std::string& p_friHost, uint16_t p_friRecvPort, uint16_t p_friSendPort)
  :m_robotName(p_robotName),
   m_rosSetJointTopic(p_rosSetJointTopic),
   m_rosGetJointTopic(p_rosGetJointTopic),
   m_rosStateTopic(p_rosStateTopic),
   m_friHost(p_friHost),
   m_friRecvPort(p_friRecvPort),
   m_friSendPort(p_friSendPort),
   m_gpi(LBR_MNJ),
   m_gpiPosCurrentBuffer(LBR_MNJ, 0),
   m_gpiPosTargetBuffer(LBR_MNJ, 0),
   m_gpiPosMinBuffer(LBR_MNJ, 0),
   m_gpiPosMaxBuffer(LBR_MNJ, 0),
   m_gpiVelCurrentBuffer(LBR_MNJ, 0),
   m_gpiVelMaxBuffer(LBR_MNJ, 0.8),
   m_gpiAccelMaxBuffer(LBR_MNJ, 10.0)
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

  // ros
  m_rosCurrentJointState.position.resize(LBR_MNJ, 0);
  m_rosCurrentJointState.velocity.resize(LBR_MNJ, 0);
  m_rosCurrentJointState.effort.resize(LBR_MNJ, 0);

  m_rosSetJointTopicSub = m_rosNode.subscribe<sensor_msgs::JointState>(m_rosSetJointTopic, 1, &RosInterpolatorFri::rosSetJointCallback, this);
  m_rosGetJointTopicPub = m_rosNode.advertise<sensor_msgs::JointState>(m_rosGetJointTopic, 1);
  m_rosStateTopicPub = m_rosNode.advertise<std_msgs::String>(m_rosStateTopic, 1);

  // fri
  memset(&m_currentFriCmdData, 0, sizeof(m_currentFriCmdData));
  m_currentFriCmdData.head.packetSize = FRI_CMD_DATA_SIZE;
  m_currentFriCmdData.head.datagramId = FRI_DATAGRAM_ID_CMD;
  m_currentFriCmdData.cmd.cmdFlags = FRI_CMD_JNTPOS;

  runFri();
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
RosInterpolatorFri::runFri()
{
  m_friIoService = new boost::asio::io_service();
  m_friSocket = new boost::asio::ip::udp::socket(*m_friIoService, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), m_friRecvPort));

  m_friResolver = new boost::asio::ip::udp::resolver(*m_friIoService);
  boost::asio::ip::udp::resolver::query sendPortQuery(boost::asio::ip::udp::v4(),
                                                      m_friHost,
                                                      ahb::string::toString(m_friSendPort));
  m_friSendEndpoint = *(m_friResolver->resolve(sendPortQuery));

  m_rosPublishThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&RosInterpolatorFri::rosPublishLoop, this)));

  friRecvStart();
  for (std::size_t i = 0; i < m_friThreadsCount; ++i) {
    boost::shared_ptr<boost::thread> thread(new boost::thread(boost::bind(&boost::asio::io_service::run, m_friIoService)));
    // Set FRI threads to real time priority
    struct sched_param param;
    param.sched_priority = 90;
    if (pthread_setschedparam((pthread_t)thread->native_handle(), SCHED_RR, &param) != 0) {
      ROS_FATAL_STREAM("Could not set FRI threads to realtime. See README.md for hints");
    }
    m_friThreads.push_back(thread);
  }
  ROS_INFO_STREAM("Started " << m_friThreads.size() << " FRI realtime threads");
}

void
RosInterpolatorFri::friRecvStart()
{
  m_friSocket->async_receive_from(boost::asio::buffer(m_friRecvBuffer),
                                      m_friRecvEndpoint,
                                      boost::bind(&RosInterpolatorFri::friRecvCallback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void
RosInterpolatorFri::friRecvCallback(const boost::system::error_code& p_error, std::size_t p_recvLength)
{
  //std::cout << "Thread " << boost::this_thread::get_id() << ": friRecvCallback()" << std::endl;

  if (p_error) {
    ROS_FATAL_STREAM("friRecvCallback: " << p_error);
    return;
  }

  if (p_recvLength != sizeof(m_lastFriMsrData)) {
    ROS_FATAL_STREAM("Received size (" << p_recvLength << ") does not match tFriMsrData size (" << sizeof(m_lastFriMsrData) << ")");
    return;
  }

  memcpy(&m_lastFriMsrData, m_friRecvBuffer.data(), sizeof(m_lastFriMsrData));
  //printFri(m_lastFriMsrData);
  friRecvStart();
  
  m_gpi.interpolate();
  m_gpi.getXNow(m_gpiPosCurrentBuffer);
  m_gpi.getVNow(m_gpiVelCurrentBuffer);
  //std::cout << "m_gpiPosCurrentBuffer: " << ahb::string::toString(m_gpiPosCurrentBuffer) << std::endl;

  // "mirror commands to get in sync", see friremote.cpp friRemote::doPositionControl
  if (m_lastFriMsrData.intf.state != FRI_STATE_CMD || m_lastFriMsrData.robot.power == 0) {
    for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
      m_currentFriCmdData.cmd.jntPos[jointIdx] = m_lastFriMsrData.data.cmdJntPos[jointIdx] + m_lastFriMsrData.data.cmdJntPosFriOffset[jointIdx];
      m_gpiPosTargetBuffer[jointIdx] = m_lastFriMsrData.data.cmdJntPos[jointIdx];
    }
    m_gpi.setXTarget(m_gpiPosTargetBuffer);
  } else {
    for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
      m_currentFriCmdData.cmd.jntPos[jointIdx] = m_gpiPosCurrentBuffer[jointIdx];
    }
  }

  m_currentFriCmdData.head.sendSeqCount++;
  m_currentFriCmdData.head.reflSeqCount = m_lastFriMsrData.head.sendSeqCount;
  //printFriCmd(m_currentFriCmdData);
  try {
    m_friSocket->send_to(boost::asio::buffer(&m_currentFriCmdData, sizeof(m_currentFriCmdData)), m_friSendEndpoint);
  } catch (boost::system::system_error const& e) {
    ROS_FATAL_STREAM("RosInterpolatorFri: Failed to send to FRI robot: " << e.what());
  }
 
  m_rosUpdateCondVar.notify_one();
}

void
RosInterpolatorFri::rosPublishLoop()
{
  boost::unique_lock<boost::mutex> lock(m_rosUpdateMutex);
  for (;;) {
    m_rosUpdateCondVar.wait(lock);
    updateRosFromFri();
    rosPublish();
  }
}

void
RosInterpolatorFri::printFri(const tFriMsrData& p_friMsrData)
{
  printf("tFriMsrData:\n");
  printf("head: sendSeqCount=%d reflSeqCount=%d\n", p_friMsrData.head.sendSeqCount, p_friMsrData.head.reflSeqCount);
  printf("intf: timestamp=%lf desiredMsrSampleTime=%1.4lf desiredCmdSampleTime=%1.4lf quality=%d state=%d\n", p_friMsrData.intf.timestamp, p_friMsrData.intf.desiredMsrSampleTime, p_friMsrData.intf.desiredCmdSampleTime, p_friMsrData.intf.quality, p_friMsrData.intf.state);
  printf("intf.stat: answerRate=%1.3lf latency=%1.3lf jitter=%1.3lf missRate=%1.3lf missCounter=%u\n", p_friMsrData.intf.stat.answerRate, p_friMsrData.intf.stat.latency, p_friMsrData.intf.stat.jitter, p_friMsrData.intf.stat.missRate, p_friMsrData.intf.stat.missCounter);
  printf("robot: power=0x%02X\n", p_friMsrData.robot.power);
  printf("data:\n");
  printf("  msrJntPos: ");
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    printf("%1.3lf ", p_friMsrData.data.msrJntPos[jointIdx]);
  }
  printf("\n");
  printf("---\n");
}

void
RosInterpolatorFri::printFriCmd(const tFriCmdData& p_friCmdData)
{
  printf("tFriCmdData:\n");
  printf("head: sendSeqCount=%d reflSeqCount=%d\n", p_friCmdData.head.sendSeqCount, p_friCmdData.head.reflSeqCount);
  printf("krl: boolData=0x%02X\n", p_friCmdData.krl.boolData);
  printf("cmd: cmdFlags=0x%02X\n", p_friCmdData.cmd.cmdFlags);
  printf("  jntPos: ");
  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    printf("%1.3lf ", p_friCmdData.cmd.jntPos[jointIdx]);
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
  //std::cout << "rosSetJointCallback: jointsMsg=" << *jointsMsg << std::endl;

  if (m_lastFriMsrData.intf.state != FRI_STATE_CMD || m_lastFriMsrData.robot.power == 0) {
    ROS_ERROR_STREAM("Robot " << m_robotName << " is not in command mode. Will not set target.");
    return;
  }


  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    if (abs(jointsMsg->position[jointIdx]) > Lwr::jointLimits.j[jointIdx]) {
      ROS_FATAL_STREAM("Joint" << jointIdx << " beyond joint limit (is=" << jointsMsg->position[jointIdx] << " limit=" << Lwr::jointLimits.j[jointIdx] << "). Will not move robot at all.");
      // TODO use m_rosStateTopicPub
      return;
    }
  }

  for (size_t jointIdx = 0; jointIdx < LBR_MNJ; jointIdx++) {
    m_gpiPosTargetBuffer[jointIdx] = jointsMsg->position[jointIdx];
  }
  m_gpi.setXTarget(m_gpiPosTargetBuffer);
}

void
RosInterpolatorFri::rosPublish()
{
  m_rosCurrentJointState.header.stamp = ros::Time::now();
  m_rosGetJointTopicPub.publish(m_rosCurrentJointState);
}
/*------------------------------------------------------------------------}}}-*/
