/**
 * ROS 2 V4L2 Camera Driver node.
 *
 * August 7, 2023
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef V4L2_CAMERA_DRIVER__V4L2_CAMERA_DRIVER_HPP
#define V4L2_CAMERA_DRIVER__V4L2_CAMERA_DRIVER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#if defined(WITH_VPI)
#include <vpi/Context.h>
#include <vpi/Image.h>
#include <vpi/LensDistortionModels.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Remap.h>
#include <vpi/algo/Rescale.h>
#elif defined(WITH_CUDA)
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include <rclcpp/rclcpp.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <image_transport/image_transport.hpp>

#include <theora_wrappers/publisher.hpp>

using namespace sensor_msgs::msg;
using namespace std_srvs::srv;

namespace v4l2_camera_driver
{

/**
 * Drives V4L2-compatible cameras with OpenCV.
 */
class V4L2CameraDriver : public dua_node::NodeBase
{
public:
  explicit V4L2CameraDriver(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~V4L2CameraDriver();

private:
  /* Node initialization routines. */
  void init_parameters();
#if defined(WITH_VPI)
  void init_vpi();
#endif

  /* Video capture device and buffers. */
  cv::VideoCapture video_cap_;
  cv::Mat frame_, frame_rot_;
  cv::Mat rectified_frame_, frame_rect_rot_;

  /* Image processing pipeline buffers and maps. */
#if defined(WITH_VPI)
  VPIBackend vpi_backend_;
  VPIStream vpi_stream_ = nullptr;
  VPIPayload vpi_remap_payload_ = nullptr, vpi_rot_payload_ = nullptr;
  VPIImage vpi_frame_ = nullptr, vpi_frame_resized_ = nullptr, vpi_frame_rot_ = nullptr;
  VPIImage vpi_frame_rect_ = nullptr, vpi_frame_rect_rot_ = nullptr;
  VPIImage vpi_frame_wrap_ = nullptr, vpi_frame_rect_wrap_ = nullptr;
  VPIImage vpi_frame_rot_wrap_ = nullptr, vpi_frame_rect_rot_wrap_ = nullptr;
  VPIWarpMap vpi_rect_map_, vpi_rot_map_;
  VPIPolynomialLensDistortionModel vpi_distortion_model_;
  VPICameraIntrinsic vpi_camera_int_;
  const VPICameraExtrinsic vpi_camera_ext_ = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0}};
#else
  cv::Mat A_, D_;
  cv::Mat map1_, map2_;
#if defined(WITH_CUDA)
  cv::cuda::GpuMat gpu_frame_, gpu_frame_rot_;
  cv::cuda::GpuMat gpu_rectified_frame_, gpu_rectified_frame_rot_;
  cv::cuda::GpuMat gpu_map1_, gpu_map2_;
#endif
#endif

  /* Node parameters. */
  int64_t camera_fps_ = 0;
  std::string camera_frame_id_;
  int64_t image_height_ = 0;
  int64_t image_rotation_ = 0;
  int64_t image_width_ = 0;

  /* Node parameters validation routines. */
  bool validate_camera_brightness(const rclcpp::Parameter & p);
  bool validate_camera_exposure(const rclcpp::Parameter & p);
  bool validate_camera_wb_temperature(const rclcpp::Parameter & p);

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr hw_enable_server_;

  /* Service callbacks. */
  void hw_enable_callback(SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr resp);

  /* image_transport publishers and buffers. */
  std::shared_ptr<image_transport::CameraPublisher> camera_pub_;
  std::shared_ptr<image_transport::Publisher> rect_pub_;
  camera_info_manager::CameraInfo camera_info_{};
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

  /* Theora stream publishers. */
  std::shared_ptr<TheoraWrappers::Publisher> stream_pub_;
  std::shared_ptr<TheoraWrappers::Publisher> rect_stream_pub_;

  /* Utility routines. */
  bool open_camera();
  void close_camera();
  bool process_frame();
  Image::SharedPtr frame_to_msg(cv::Mat & frame);

  /* Camera sampling thread. */
  std::thread camera_sampling_thread_;
  void camera_sampling_routine();

  /* Synchronization primitives. */
  std::atomic<bool> stopped_;
};

} // namespace v4l2_camera_driver

#endif // V4L2_CAMERA_DRIVER__V4L2_CAMERA_DRIVER_HPP
