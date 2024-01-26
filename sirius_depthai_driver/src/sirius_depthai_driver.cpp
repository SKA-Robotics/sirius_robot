#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <bits/stdc++.h>
#include <functional>
#include <iostream>
#include <tuple>
#include <vector>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <depthai_ros_msgs/ImuWithMagneticField.h>

std::vector<std::string> usbStrings = { "UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS" };

auto getMonoResolution(std::string resolution)
{
  typedef dai::node::MonoCamera::Properties::SensorResolution monoResolution;
  if (resolution == "720p")
  {
    return std::make_tuple(1280, 720, monoResolution::THE_720_P);
  }
  else if (resolution == "400p")
  {
    return std::make_tuple(640, 400, monoResolution::THE_400_P);
  }
  else if (resolution == "800p")
  {
    return std::make_tuple(1280, 800, monoResolution::THE_800_P);
  }
  else if (resolution == "480p")
  {
    return std::make_tuple(640, 480, monoResolution::THE_480_P);
  }
  else
  {
    ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
    throw std::runtime_error("Invalid mono camera resolution.");
  }
}
auto getRGBResolution(std::string resolution)
{
  typedef dai::node::ColorCamera::Properties::SensorResolution rgbResolution;
  if (resolution == "1080p")
  {
    return std::make_tuple(1920, 1080, rgbResolution::THE_1080_P);
  }
  else if (resolution == "4K")
  {
    return std::make_tuple(3840, 2160, rgbResolution::THE_4_K);
  }
  else if (resolution == "12MP")
  {
    return std::make_tuple(4056, 3040, rgbResolution::THE_12_MP);
  }
  else if (resolution == "13MP")
  {
    return std::make_tuple(4208, 3120, rgbResolution::THE_13_MP);
  }
  else
  {
    ROS_ERROR("Invalid parameter. -> rgbResolution: %s", resolution.c_str());
    throw std::runtime_error("Invalid color camera resolution.");
  }
}
auto addMonoPipeline(dai::Pipeline& pipeline, int fps, dai::node::MonoCamera::Properties::SensorResolution resolution,
                     std::string name, dai::CameraBoardSocket socket)
{
  auto monoCamera = pipeline.create<dai::node::MonoCamera>();

  monoCamera->setResolution(resolution);
  monoCamera->setBoardSocket(socket);
  monoCamera->setFps(fps);

  return monoCamera;
}
void addRGBPipeline(dai::Pipeline& pipeline, int fps, dai::node::ColorCamera::Properties::SensorResolution resolution,
                    std::string name, dai::CameraBoardSocket socket)
{
  auto rgbCamera = pipeline.create<dai::node::ColorCamera>();
  auto xOut = pipeline.create<dai::node::XLinkOut>();
  xOut->setStreamName(name);

  rgbCamera->setResolution(resolution);
  rgbCamera->setBoardSocket(socket);
  rgbCamera->setFps(fps);

  rgbCamera->isp.link(xOut->input);
}
auto addDepthPipeline(dai::Pipeline& pipeline, int fps, dai::node::ColorCamera::Properties::SensorResolution resolution,
                      std::string name, dai::CameraBoardSocket socket)
{
  auto stereo = pipeline.create<dai::node::StereoDepth>();

  stereo->initialConfig.setConfidenceThreshold(200);
  stereo->setRectifyEdgeFillColor(0);
  stereo->initialConfig.setLeftRightCheckThreshold(5);
  stereo->setLeftRightCheck(true);
  stereo->setExtendedDisparity(false);
  stereo->setSubpixel(true);
  stereo->setRectifyEdgeFillColor(0);

  return stereo;
}

std::tuple<std::vector<dai::IMUSensor>, bool, dai::IMUSensor> getImuEnum(std::string rotationVectorType,
                                                                         std::string accelerometerType,
                                                                         std::string gyroscopeType,
                                                                         std::string magnetometerType)
{
  std::vector<dai::IMUSensor> imuEnums;
  dai::IMUSensor magnetometerEnum;
  bool magnetometerEnabled = false;

  std::map<std::string, dai::IMUSensor> rotationVectorMap = {
    { "ROTATION_VECTOR", dai::IMUSensor::ROTATION_VECTOR },
    { "GAME_ROTATION_VECTOR", dai::IMUSensor::GAME_ROTATION_VECTOR },
    { "GEOMAGNETIC_ROTATION_VECTOR", dai::IMUSensor::GEOMAGNETIC_ROTATION_VECTOR },
    { "ARVR_STABILIZED_ROTATION_VECTOR", dai::IMUSensor::ARVR_STABILIZED_ROTATION_VECTOR },
    { "ARVR_STABILIZED_GAME_ROTATION_VECTOR", dai::IMUSensor::ARVR_STABILIZED_GAME_ROTATION_VECTOR }
  };

  std::map<std::string, dai::IMUSensor> accelerometerMap = { { "ACCELEROMETER_RAW", dai::IMUSensor::ACCELEROMETER_RAW },
                                                             { "ACCELEROMETER", dai::IMUSensor::ACCELEROMETER },
                                                             { "LINEAR_ACCELERATION",
                                                               dai::IMUSensor::LINEAR_ACCELERATION },
                                                             { "GRAVITY", dai::IMUSensor::GRAVITY } };

  std::map<std::string, dai::IMUSensor> gyroscopeMap = {
    { "GYROSCOPE_RAW", dai::IMUSensor::GYROSCOPE_RAW },
    { "GYROSCOPE_CALIBRATED", dai::IMUSensor::GYROSCOPE_CALIBRATED },
    { "GYROSCOPE_UNCALIBRATED", dai::IMUSensor::GYROSCOPE_UNCALIBRATED }
  };

  std::map<std::string, dai::IMUSensor> magnetometerMap = {
    { "MAGNETOMETER_RAW", dai::IMUSensor::MAGNETOMETER_RAW },
    { "MAGNETOMETER_CALIBRATED", dai::IMUSensor::MAGNETOMETER_CALIBRATED },
    { "MAGNETOMETER_UNCALIBRATED", dai::IMUSensor::MAGNETOMETER_UNCALIBRATED }
  };

  if (rotationVectorMap.find(rotationVectorType) != rotationVectorMap.end())
  {
    imuEnums.push_back(rotationVectorMap[rotationVectorType]);
  }
  if (gyroscopeMap.find(gyroscopeType) != gyroscopeMap.end())
  {
    imuEnums.push_back(gyroscopeMap[gyroscopeType]);
  }
  if (accelerometerMap.find(accelerometerType) != accelerometerMap.end())
  {
    imuEnums.push_back(accelerometerMap[accelerometerType]);
  }
  if (magnetometerMap.find(magnetometerType) != magnetometerMap.end())
  {
    magnetometerEnabled = true;
    magnetometerEnum = magnetometerMap[magnetometerType];
  }
  return std::make_tuple(imuEnums, magnetometerEnabled, magnetometerEnum);
}
void addIMUPipeline(dai::Pipeline& pipeline, int rate, int magnetometer_rate, std::string name,
                    std::vector<dai::IMUSensor> imuEnums, bool magnetometerEnabled, dai::IMUSensor magnetometerEnum)
{
  auto imu = pipeline.create<dai::node::IMU>();
  auto xOut = pipeline.create<dai::node::XLinkOut>();
  xOut->setStreamName(name);
  if (imuEnums.size() > 0)
  {
    imu->enableIMUSensor(imuEnums, rate);
  }

  if (magnetometerEnabled)
  {
    imu->enableIMUSensor(magnetometerEnum, magnetometer_rate);
  }
  imu->setBatchReportThreshold(5);
  imu->setMaxBatchReports(20);

  imu->out.link(xOut->input);
}
std::tuple<dai::Pipeline, int, int> createPipeline(int fps, int imu_rate, int magnetometer_rate,
                                                   std::string stereoResolution, std::string colorResolution,
                                                   std::vector<dai::IMUSensor> imuEnums, bool magnetometerEnabled,
                                                   dai::IMUSensor magnetometerEnum)
{
  dai::Pipeline pipeline;
  pipeline.setXLinkChunkSize(0);

  dai::node::MonoCamera::Properties::SensorResolution monoResolution;
  int stereoWidth, stereoHeight;
  std::tie(stereoWidth, stereoHeight, monoResolution) = getMonoResolution(stereoResolution);
  auto rgbResolution = std::get<2>(getRGBResolution(colorResolution));

  auto left_pipe = addMonoPipeline(pipeline, fps, monoResolution, "left", dai::CameraBoardSocket::CAM_B);
  auto right_pipe = addMonoPipeline(pipeline, fps, monoResolution, "right", dai::CameraBoardSocket::CAM_C);
  addRGBPipeline(pipeline, fps, rgbResolution, "rgb", dai::CameraBoardSocket::CAM_A);
  auto depth_pipe = addDepthPipeline(pipeline, fps, rgbResolution, "depth", dai::CameraBoardSocket::CAM_A);
  addIMUPipeline(pipeline, imu_rate, magnetometer_rate, "imu", imuEnums, magnetometerEnabled, magnetometerEnum);
  auto xOutLeft = pipeline.create<dai::node::XLinkOut>();
  auto xOutRight = pipeline.create<dai::node::XLinkOut>();
  xOutLeft->setStreamName("left");
  xOutRight->setStreamName("right");

  left_pipe->out.link(depth_pipe->left);
  right_pipe->out.link(depth_pipe->right);

  auto xOutDepth = pipeline.create<dai::node::XLinkOut>();
  xOutDepth->setStreamName("depth");
  depth_pipe->depth.link(xOutDepth->input);
  depth_pipe->syncedLeft.link(xOutLeft->input);
  depth_pipe->syncedRight.link(xOutRight->input);

  return std::make_tuple(pipeline, stereoWidth, stereoHeight);
}
std::shared_ptr<dai::Device> connect(ros::NodeHandle& nh, dai::Pipeline& pipeline, std::string mxId)
{
  bool isDeviceFound = false;
  std::shared_ptr<dai::Device> device;
  std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

  std::cout << "Listing available devices..." << std::endl;
  for (auto deviceInfo : availableDevices)
  {
    std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
    if (deviceInfo.getMxId() == mxId)
    {
      if (deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER)
      {
        isDeviceFound = true;
        device = std::make_shared<dai::Device>(pipeline, deviceInfo);
        break;
      }
      else if (deviceInfo.state == X_LINK_BOOTED)
      {
        throw std::runtime_error("ros::NodeHandle() from Node \"" + nh.getNamespace() +
                                 "\" DepthAI Device with MxId  \"" + mxId +
                                 "\" is already booted on different process.  \"");
      }
    }
    else if (mxId.empty())
    {
      isDeviceFound = true;
      device = std::make_shared<dai::Device>(pipeline, deviceInfo);
      break;
    }
  }

  if (!isDeviceFound)
  {
    throw std::runtime_error("ros::NodeHandle() from Node \"" + nh.getNamespace() + "\" DepthAI Device with MxId  \"" +
                             mxId + "\" not found.  \"");
  }

  std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
  return device;
}

void imuMagQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data,
               dai::rosBridge::ImuConverter& imuConverter, const ros::Publisher& rosImuPub,
               const ros::Publisher& magPub, bool imuEnabled, bool magnetometerEnabled)
{
  auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
  std::deque<depthai_ros_msgs::ImuWithMagneticField> deq;
  imuConverter.toRosDaiMsg(imuData, deq);
  while (deq.size() > 0)
  {
    auto currMsg = deq.front();
    if (!imuEnabled)
    {
      sensor_msgs::Imu imu = currMsg.imu;
      imu.header = currMsg.header;
      rosImuPub.publish(imu);
    }
    if (magnetometerEnabled)
    {
      sensor_msgs::MagneticField field = currMsg.field;
      field.header = currMsg.header;
      magPub.publish(field);
    }
    deq.pop_front();
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "oak-d_w");
  ros::NodeHandle nh("~");

  std::string tf, mxId, stereoResolution = "720p", rgbResolution = "1080p", rotationVectorType = "ROTATION_VECTOR",
                        accelerometerType = "ACCELEROMETER", gyroscopeType = "GYROSCOPE_CALIBRATED",
                        magnetometerType = "MAGNETOMETER_CALIBRATED", left_camera_param_uri = "",
                        right_camera_param_uri = "", rgb_camera_param_uri = "";
  int badParams = 0, fps = 30, imu_rate = 200, magnetometer_rate = 100;
  double angularVelCovariance, linearAccelCovariance, rotationCovariance, magneticCovariance;

  badParams += !nh.getParam("mxId", mxId);
  badParams += !nh.getParam("tf", tf);
  badParams += !nh.getParam("imu_rate", imu_rate);
  badParams += !nh.getParam("magnetometer_rate", magnetometer_rate);
  badParams += !nh.getParam("fps", fps);
  badParams += !nh.getParam("stereoResolution", stereoResolution);
  badParams += !nh.getParam("rgbResolution", rgbResolution);
  badParams += !nh.getParam("angularVelCovariance", angularVelCovariance);
  badParams += !nh.getParam("linearAccelCovariance", linearAccelCovariance);
  badParams += !nh.getParam("rotationCovariance", rotationCovariance);
  badParams += !nh.getParam("magneticCovariance", magneticCovariance);
  nh.getParam("rotationVectorType", rotationVectorType);
  nh.getParam("accelerometerType", accelerometerType);
  nh.getParam("gyroscopeType", gyroscopeType);
  nh.getParam("magnetometerType", magnetometerType);
  nh.getParam("left_camera_param_uri", left_camera_param_uri);
  nh.getParam("right_camera_param_uri", right_camera_param_uri);
  nh.getParam("rgb_camera_param_uri", rgb_camera_param_uri);

  if (badParams > 0)
  {
    std::cout << " Bad parameters -> " << badParams << std::endl;
    throw std::runtime_error("Couldn't find %d of the parameters");
  }

  std::vector<dai::IMUSensor> imuEnums;
  dai::IMUSensor magnetometerEnum;
  bool magnetometerEnabled = false;
  std::tie(imuEnums, magnetometerEnabled, magnetometerEnum) =
      getImuEnum(rotationVectorType, accelerometerType, gyroscopeType, magnetometerType);

  dai::Pipeline pipeline;
  int width, height;
  std::tie(pipeline, width, height) = createPipeline(fps, imu_rate, magnetometer_rate, stereoResolution, rgbResolution,
                                                     imuEnums, magnetometerEnabled, magnetometerEnum);
  auto device = connect(nh, pipeline, mxId);
  auto calibrationHandler = device->readCalibration();

  dai::rosBridge::ImageConverter converter(tf + "_left_camera_optical_frame", true);
  dai::rosBridge::ImageConverter rightconverter(tf + "_right_camera_optical_frame", true);
  dai::rosBridge::ImageConverter rgbConverter(tf + "_rgb_camera_optical_frame", false);
  dai::rosBridge::ImuConverter imuConverter(tf + "_imu_frame", dai::ros::ImuSyncMethod::COPY, linearAccelCovariance,
                                            angularVelCovariance, rotationCovariance, magneticCovariance, true);

  auto leftCameraInfo =
      converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, width, height);
  auto rightCameraInfo =
      converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
  auto rgbCameraInfo =
      rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);

  auto leftQueue = device->getOutputQueue("left", 30, false);
  auto rightQueue = device->getOutputQueue("right", 30, false);
  auto imgQueue = device->getOutputQueue("rgb", 30, false);
  auto imuQueue = device->getOutputQueue("imu", 30, false);
  auto depthQueue = device->getOutputQueue("depth", 30, false);

  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish;
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> rightPublish;
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> rgbPublish;
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> depthPublish;

  if (left_camera_param_uri.empty())
  {
    leftPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        leftQueue, nh, "left/image", [&](auto in, auto& out) { converter.toRosMsg(in, out); }, 30, leftCameraInfo,
        "left");
  }
  else
  {
    leftPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        leftQueue, nh, "left/image", [&](auto in, auto& out) { converter.toRosMsg(in, out); }, 30,
        left_camera_param_uri, "left");
  }

  if (right_camera_param_uri.empty())
  {
    rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        rightQueue, nh, "right/image", [&](auto in, auto& out) { rightconverter.toRosMsg(in, out); }, 30,
        rightCameraInfo, "right");
  }
  else
  {
    rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        rightQueue, nh, "right/image", [&](auto in, auto& out) { rightconverter.toRosMsg(in, out); }, 30,
        right_camera_param_uri, "right");
  }

  if (rgb_camera_param_uri.empty())
  {
    rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        imgQueue, nh, std::string("rgb/image_color"), [&](auto in, auto& out) { rgbConverter.toRosMsg(in, out); }, 30,
        rgbCameraInfo, "rgb");
    depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        depthQueue, nh, std::string("depth_registered/image_raw"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &rgbConverter,  // since the converter has the same frame name
                                  // and image type is also same we can reuse it
                  std::placeholders::_1, std::placeholders::_2),
        30, rgbCameraInfo, "depth");
  }
  else
  {
    rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        imgQueue, nh, std::string("rgb/image_color"), [&](auto in, auto& out) { rgbConverter.toRosMsg(in, out); }, 30,
        rgb_camera_param_uri, "rgb");
    depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
        depthQueue, nh, std::string("depth_registered/image_raw"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1,
                  std::placeholders::_2),
        30, rgb_camera_param_uri, "depth");
  }

  ros::Publisher rosImuPub;
  ros::Publisher magPub;
  if (!imuEnums.empty())
  {
    rosImuPub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
  }
  if (magnetometerEnabled)
  {
    magPub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
  }

  leftPublish->addPublisherCallback();
  rightPublish->addPublisherCallback();
  rgbPublish->addPublisherCallback();
  imuQueue->addCallback([&imuConverter, &rosImuPub, &magPub, imuEnabled = imuEnums.empty(),
                         magnetometerEnabled](const auto& name, const auto& data) {
    imuMagQCB(name, data, imuConverter, rosImuPub, magPub, imuEnabled, magnetometerEnabled);
  });
  depthPublish->addPublisherCallback();

  ros::spin();
  return 0;
}