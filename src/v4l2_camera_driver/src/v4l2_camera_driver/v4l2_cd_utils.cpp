/**
 * ROS 2 V4L2 Camera Driver node auxiliary routines.
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

#include <cfloat>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include <v4l2_camera_driver/v4l2_camera_driver.hpp>

namespace v4l2_camera_driver
{

/**
 * @brief Opens the camera.
 *
 * @return True if the camera was opened successfully, false otherwise.
 */
bool V4L2CameraDriver::open_camera()
{
  // Open capture device
  bool opened = false;
  std::string camera_device_file = this->get_parameter("camera_device_file").as_string();
  if (camera_device_file.empty()) {
    int64_t camera_id = this->get_parameter("camera_id").as_int();
    RCLCPP_INFO(
      this->get_logger(),
      "Opening camera with ID: %ld",
      camera_id);
    opened = video_cap_.open(camera_id, cv::CAP_V4L2);
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Opening camera from device file: %s",
      camera_device_file.c_str());
    opened = video_cap_.open(camera_device_file, cv::CAP_V4L2);
  }
  if (!opened ||
    !video_cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_) ||
    !video_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_) ||
    !video_cap_.set(cv::CAP_PROP_FPS, camera_fps_))
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "V4L2CameraDriver::open_camera: Failed to open capture device");
    return false;
  }

  // Set camera parameters
  double exposure = this->get_parameter("camera_exposure").as_double();
  double brightness = this->get_parameter("camera_brightness").as_double();
  double wb_temperature = this->get_parameter("camera_wb_temperature").as_double();
  bool success;
  if (exposure != 0.0) {
    success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::open_camera: cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 0.75) failed");
      return false;
    }
    success = video_cap_.set(cv::CAP_PROP_EXPOSURE, exposure);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::open_camera: cv::VideoCapture::set(CAP_PROP_EXPOSURE) failed");
      return false;
    }
  }
  if (brightness != 0.0) {
    success = video_cap_.set(cv::CAP_PROP_BRIGHTNESS, brightness);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::open_camera: cv::VideoCapture::set(CAP_PROP_BRIGHTNESS) failed");
      return false;
    }
  }
  if (wb_temperature == 0.0) {
    success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 1.0);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::open_camera: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 1.0) failed");
      return false;
    }
  } else {
    success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 0.0);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::open_camera: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 0.0) failed");
      return false;
    }
    success = video_cap_.set(cv::CAP_PROP_WB_TEMPERATURE, wb_temperature);
    if (!success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::open_camera: cv::VideoCapture::set(CAP_PROP_WB_TEMPERATURE) failed");
      return false;
    }
  }

  return true;
}

/**
 * @brief Closes the camera.
 */
void V4L2CameraDriver::close_camera()
{
  if (video_cap_.isOpened()) {
    video_cap_.release();
  }
}

#if defined(WITH_VPI)
/**
 * @brief Initializes the VPI context and data.
 */
void V4L2CameraDriver::init_vpi()
{
  VPIStatus err;

  // Create VPI stream
  err = vpiStreamCreate(
    vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA,
    &vpi_stream_);
  if (err != VPIStatus::VPI_SUCCESS) {
    RCLCPP_FATAL(this->get_logger(), "V4L2CameraDriver::init_vpi: Failed to create VPI stream");
    throw std::runtime_error("V4L2CameraDriver::init_vpi: Failed to create VPI stream");
  }

  if (cinfo_manager_->isCalibrated()) {
    // Initialize rectification map region as the whole image
    memset(&vpi_rect_map_, 0, sizeof(vpi_rect_map_));
    vpi_rect_map_.grid.numHorizRegions = 1;
    vpi_rect_map_.grid.numVertRegions = 1;
    vpi_rect_map_.grid.regionWidth[0] = image_width_;
    vpi_rect_map_.grid.regionHeight[0] = image_height_;
    vpi_rect_map_.grid.horizInterval[0] = 1;
    vpi_rect_map_.grid.vertInterval[0] = 1;
    err = vpiWarpMapAllocData(&vpi_rect_map_);
    if (err != VPI_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "V4L2CameraDriver::init_vpi: Failed to allocate rectification warp map");
      throw std::runtime_error(
              "V4L2CameraDriver::init_vpi: Failed to allocate rectification warp map");
    }

    // Get intrinsic camera parameters
    vpi_camera_int_[0][0] = float(camera_info_.k[0]);
    vpi_camera_int_[0][2] = float(camera_info_.k[2]);
    vpi_camera_int_[1][1] = float(camera_info_.k[4]);
    vpi_camera_int_[1][2] = float(camera_info_.k[5]);

    // Set "plumb_bob" distortion model coefficients
    memset(&vpi_distortion_model_, 0, sizeof(vpi_distortion_model_));
    vpi_distortion_model_.k1 = float(camera_info_.d[0]);
    vpi_distortion_model_.k2 = float(camera_info_.d[1]);
    vpi_distortion_model_.k3 = float(camera_info_.d[4]);
    vpi_distortion_model_.k4 = 0.0f;
    vpi_distortion_model_.k5 = 0.0f;
    vpi_distortion_model_.k6 = 0.0f;
    vpi_distortion_model_.p1 = float(camera_info_.d[2]);
    vpi_distortion_model_.p2 = float(camera_info_.d[3]);

    // Create warp map from distortion model
    err = vpiWarpMapGenerateFromPolynomialLensDistortionModel(
      vpi_camera_int_,
      vpi_camera_ext_,
      vpi_camera_int_,
      &vpi_distortion_model_,
      &vpi_rect_map_);
    if (err != VPIStatus::VPI_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "V4L2CameraDriver::init_vpi: Failed to generate VPI rectification map");
      throw std::runtime_error(
              "V4L2CameraDriver::init_vpi: Failed to generate VPI rectification map");
    }

    // Create VPI remap payload
    err = vpiCreateRemap(
      vpi_backend_,
      &vpi_rect_map_,
      &vpi_remap_payload_);
    if (err != VPIStatus::VPI_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(), "V4L2CameraDriver::init_vpi: Failed to create VPI remap payload");
      throw std::runtime_error("V4L2CameraDriver::init_vpi: Failed to create VPI remap payload");
    }
  }

  // Initialize rotation data and warp map
  int32_t rot_width = 0, rot_height = 0;
  if (image_rotation_ != 0) {
    if (image_rotation_ == 90 || image_rotation_ == -90) {
      rot_width = image_height_;
      rot_height = image_width_;
    } else {
      // +/-180
      rot_width = image_width_;
      rot_height = image_height_;
    }

    memset(&vpi_rot_map_, 0, sizeof(vpi_rot_map_));
    vpi_rot_map_.grid.numHorizRegions = 1;
    vpi_rot_map_.grid.numVertRegions = 1;
    vpi_rot_map_.grid.regionWidth[0] = rot_width;
    vpi_rot_map_.grid.regionHeight[0] = rot_height;
    vpi_rot_map_.grid.horizInterval[0] = 1;
    vpi_rot_map_.grid.vertInterval[0] = 1;
    err = vpiWarpMapAllocData(&vpi_rot_map_);
    if (err != VPI_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "V4L2CameraDriver::init_vpi: Failed to allocate rotation warp map");
      throw std::runtime_error("V4L2CameraDriver::init_vpi: Failed to allocate rotation warp map");
    }

    err = vpiWarpMapGenerateIdentity(&vpi_rot_map_);
    if (err != VPI_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "V4L2CameraDriver::init_vpi: Failed to generate rotation warp map");
      throw std::runtime_error("V4L2CameraDriver::init_vpi: Failed to generate rotation warp map");
    }

    for (int i = 0; i < vpi_rot_map_.numVertPoints; ++i) {
      VPIKeypointF32 * row =
        (VPIKeypointF32 *)((uint8_t *)vpi_rot_map_.keypoints + vpi_rot_map_.pitchBytes * i);
      for (int j = 0; j < vpi_rot_map_.numHorizPoints; ++j) {
        if (image_rotation_ == 90) {
          row[j].x = float(image_width_ - i - 1);
          row[j].y = float(j);
        } else if (image_rotation_ == -90) {
          row[j].x = float(i);
          row[j].y = float(image_height_ - j - 1);
        } else {
          // +/-180
          row[j].x = float(image_width_ - j - 1);
          row[j].y = float(image_height_ - i - 1);
        }
      }
    }

    err = vpiCreateRemap(
      vpi_backend_,
      &vpi_rot_map_,
      &vpi_rot_payload_);
    if (err != VPI_SUCCESS) {
      RCLCPP_FATAL(
        this->get_logger(),
        "V4L2CameraDriver::init_vpi: Failed to create rotation remap payload");
      throw std::runtime_error(
              "V4L2CameraDriver::init_vpi: Failed to create rotation remap payload");
    }
  }

  // Create VPI image buffers
  VPIStatus err_img1, err_img2, err_img3 = VPIStatus::VPI_SUCCESS;
  VPIStatus err_img4 = VPIStatus::VPI_SUCCESS, err_img5 = VPIStatus::VPI_SUCCESS;
  err_img1 = vpiImageCreate(
    image_width_,
    image_height_,
    VPI_IMAGE_FORMAT_NV12_ER,
    vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
    &vpi_frame_);
  err_img2 = vpiImageCreate(
    image_width_,
    image_height_,
    VPI_IMAGE_FORMAT_NV12_ER,
    vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
    &vpi_frame_resized_);
  if (cinfo_manager_->isCalibrated()) {
    err_img3 = vpiImageCreate(
      image_width_,
      image_height_,
      VPI_IMAGE_FORMAT_NV12_ER,
      vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
      &vpi_frame_rect_);
  }
  if (image_rotation_ != 0) {
    err_img4 = vpiImageCreate(
      rot_width,
      rot_height,
      VPI_IMAGE_FORMAT_NV12_ER,
      vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
      &vpi_frame_rot_);
    if (cinfo_manager_->isCalibrated()) {
      err_img5 = vpiImageCreate(
        rot_width,
        rot_height,
        VPI_IMAGE_FORMAT_NV12_ER,
        vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
        &vpi_frame_rect_rot_);
    }
  }
  if (err_img1 != VPIStatus::VPI_SUCCESS ||
    err_img2 != VPIStatus::VPI_SUCCESS ||
    err_img3 != VPIStatus::VPI_SUCCESS ||
    err_img4 != VPIStatus::VPI_SUCCESS ||
    err_img5 != VPIStatus::VPI_SUCCESS)
  {
    RCLCPP_FATAL(this->get_logger(), "V4L2CameraDriver::init_vpi: Failed to create VPI images");
    throw std::runtime_error("V4L2CameraDriver::init_vpi: Failed to create VPI images");
  }
}
#endif

/**
 * @brief Performs the image processing steps, between frame acquisition and messages composition.
 *
 * @return True if the frame was processed successfully, false otherwise.
 */
bool V4L2CameraDriver::process_frame()
{
  /**
   * The following code supports the following APIs, and self-compiles according to the
   * availability of the APIs on the system it is compiled on:
   * - OpenCV (CPU)
   * - OpenCV (CUDA)
   * - Nvidia VPI
   *
   * Independently of the APIs being used, the code is structured in the following way:
   * - Get the frame to process.
   * - Resize it to the desired format (usually not done by the camera, but we try to request it).
   * - If the camera has been calibrated, rectify the frame.
   * - If a rotation has been requested, rotate the frame and the rectified frame.
   * - Write final frames.
   */

#if defined(WITH_VPI)
  VPIStatus err;

  // Wrap the cv::Mat to a VPIImage (input, output)
  if (vpi_frame_wrap_ == nullptr) {
    err = vpiImageCreateWrapperOpenCVMat(
      frame_,
      vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
      &vpi_frame_wrap_);
  } else {
    err = vpiImageSetWrappedOpenCVMat(
      vpi_frame_wrap_,
      frame_);
  }
  if (err != VPIStatus::VPI_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "V4L2CameraDriver::process_frame: Failed to wrap cv::Mat to VPIImage");
    return false;
  }

  // Wrap the rectified cv::Mat to a VPIImage (output)
  if (cinfo_manager_->isCalibrated()) {
    if (vpi_frame_rect_wrap_ == nullptr) {
      rectified_frame_ = cv::Mat(image_height_, image_width_, CV_8UC3);
      err = vpiImageCreateWrapperOpenCVMat(
        rectified_frame_,
        vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
        &vpi_frame_rect_wrap_);
    } else {
      err = vpiImageSetWrappedOpenCVMat(
        vpi_frame_rect_wrap_,
        rectified_frame_);
    }
    if (err != VPIStatus::VPI_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::process_frame: Failed to wrap rectified cv::Mat to VPIImage");
      return false;
    }
  }

  // Wrap the rotated cv::Mat to a VPIImage (output)
  if (image_rotation_ != 0) {
    if (vpi_frame_rot_wrap_ == nullptr) {
      int32_t rot_width = 0, rot_height = 0;
      vpiImageGetSize(vpi_frame_rot_, &rot_width, &rot_height);
      frame_rot_ = cv::Mat(rot_height, rot_width, CV_8UC3);
      err = vpiImageCreateWrapperOpenCVMat(
        frame_rot_,
        vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
        &vpi_frame_rot_wrap_);
      if (err != VPIStatus::VPI_SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::process_frame: Failed to wrap rotated cv::Mat to VPIImage");
        return false;
      }
    }
    if (cinfo_manager_->isCalibrated() && vpi_frame_rect_rot_wrap_ == nullptr) {
      int32_t rot_width = 0, rot_height = 0;
      vpiImageGetSize(vpi_frame_rect_rot_, &rot_width, &rot_height);
      frame_rect_rot_ = cv::Mat(rot_height, rot_width, CV_8UC3);
      err = vpiImageCreateWrapperOpenCVMat(
        frame_rect_rot_,
        vpi_backend_ | VPIBackend::VPI_BACKEND_CUDA | VPI_EXCLUSIVE_STREAM_ACCESS,
        &vpi_frame_rect_rot_wrap_);
      if (err != VPIStatus::VPI_SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::process_frame: Failed to wrap rotated rectified cv::Mat to VPIImage");
        return false;
      }
    }
  }

  // Convert frame from BGR to NV12
  err = vpiSubmitConvertImageFormat(
    vpi_stream_,
    VPIBackend::VPI_BACKEND_CUDA,
    vpi_frame_wrap_,
    vpi_frame_,
    NULL);
  if (err != VPIStatus::VPI_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "V4L2CameraDriver::process_frame: Failed to convert image BGR->NV12");
    return false;
  }

  // Rescale frame to desired size
  err = vpiSubmitRescale(
    vpi_stream_,
    VPIBackend::VPI_BACKEND_CUDA,
    vpi_frame_,
    vpi_frame_resized_,
    VPIInterpolationType::VPI_INTERP_LINEAR,
    VPIBorderExtension::VPI_BORDER_ZERO,
    0);
  if (err != VPIStatus::VPI_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "V4L2CameraDriver::process_frame: Failed to rescale image");
    return false;
  }

  // Rectify frame
  if (cinfo_manager_->isCalibrated()) {
    err = vpiSubmitRemap(
      vpi_stream_,
      vpi_backend_,
      vpi_remap_payload_,
      vpi_frame_resized_,
      vpi_frame_rect_,
      VPIInterpolationType::VPI_INTERP_LINEAR,
      VPIBorderExtension::VPI_BORDER_ZERO,
      0);
    if (err != VPIStatus::VPI_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::process_frame: Failed to rectify image");
      return false;
    }
  }

  // Rotate frames
  if (image_rotation_ != 0) {
    err = vpiSubmitRemap(
      vpi_stream_,
      vpi_backend_,
      vpi_rot_payload_,
      vpi_frame_resized_,
      vpi_frame_rot_,
      VPIInterpolationType::VPI_INTERP_LINEAR,
      VPIBorderExtension::VPI_BORDER_ZERO,
      0);
    if (err != VPIStatus::VPI_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::process_frame: Failed to rotate image");
      return false;
    }

    if (cinfo_manager_->isCalibrated()) {
      err = vpiSubmitRemap(
        vpi_stream_,
        vpi_backend_,
        vpi_rot_payload_,
        vpi_frame_rect_,
        vpi_frame_rect_rot_,
        VPIInterpolationType::VPI_INTERP_LINEAR,
        VPIBorderExtension::VPI_BORDER_ZERO,
        0);
      if (err != VPIStatus::VPI_SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::process_frame: Failed to rotate rectified image");
        return false;
      }
    }
  }

  // Convert frames back to BGR
  if (image_rotation_ != 0) {
    err = vpiSubmitConvertImageFormat(
      vpi_stream_,
      VPIBackend::VPI_BACKEND_CUDA,
      vpi_frame_rot_,
      vpi_frame_rot_wrap_,
      NULL);
    if (err != VPIStatus::VPI_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::process_frame: Failed to convert rotated image NV12->BGR");
      return false;
    }

    if (cinfo_manager_->isCalibrated()) {
      err = vpiSubmitConvertImageFormat(
        vpi_stream_,
        VPIBackend::VPI_BACKEND_CUDA,
        vpi_frame_rect_rot_,
        vpi_frame_rect_rot_wrap_,
        NULL);
      if (err != VPIStatus::VPI_SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::process_frame: Failed to convert rectified rotated image NV12->BGR");
        return false;
      }
    }
  } else {
    err = vpiSubmitConvertImageFormat(
      vpi_stream_,
      VPIBackend::VPI_BACKEND_CUDA,
      vpi_frame_resized_,
      vpi_frame_wrap_,
      NULL);
    if (err != VPIStatus::VPI_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "V4L2CameraDriver::process_frame: Failed to convert image NV12->BGR");
      return false;
    }

    if (cinfo_manager_->isCalibrated()) {
      err = vpiSubmitConvertImageFormat(
        vpi_stream_,
        VPIBackend::VPI_BACKEND_CUDA,
        vpi_frame_rect_,
        vpi_frame_rect_wrap_,
        NULL);
      if (err != VPIStatus::VPI_SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::process_frame: Failed to convert rectified image NV12->BGR");
        return false;
      }
    }
  }

  // Wait for the stream to finish
  err = vpiStreamSync(vpi_stream_);
  if (err != VPIStatus::VPI_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(),
      "V4L2CameraDriver::process_frame: VPIStream processing failed");
    return false;
  }

  return true;
#elif defined(WITH_CUDA)
  gpu_frame_.upload(frame_);

  cv::cuda::resize(gpu_frame_, gpu_frame_, cv::Size(image_width_, image_height_));

  if (cinfo_manager_->isCalibrated()) {
    cv::cuda::remap(
      gpu_frame_,
      gpu_rectified_frame_,
      gpu_map1_,
      gpu_map2_,
      cv::InterpolationFlags::INTER_LINEAR,
      cv::BorderTypes::BORDER_CONSTANT);
  }

  if (image_rotation_ == 90) {
    cv::cuda::rotate(
      gpu_frame_,
      gpu_frame_rot_,
      cv::Size(image_height_, image_width_),
      90.0,
      0.0,
      image_width_ - 1,
      cv::InterpolationFlags::INTER_LINEAR);
    if (cinfo_manager_->isCalibrated()) {
      cv::cuda::rotate(
        gpu_rectified_frame_,
        gpu_rectified_frame_rot_,
        cv::Size(image_height_, image_width_),
        90.0,
        0.0,
        image_width_ - 1,
        cv::InterpolationFlags::INTER_LINEAR);
    }
  } else if (image_rotation_ == -90) {
    cv::cuda::rotate(
      gpu_frame_,
      gpu_frame_rot_,
      cv::Size(image_height_, image_width_),
      -90.0,
      image_height_ - 1,
      0.0,
      cv::InterpolationFlags::INTER_LINEAR);
    if (cinfo_manager_->isCalibrated()) {
      cv::cuda::rotate(
        gpu_rectified_frame_,
        gpu_rectified_frame_rot_,
        cv::Size(image_height_, image_width_),
        -90.0,
        image_height_ - 1,
        0.0,
        cv::InterpolationFlags::INTER_LINEAR);
    }
  } else if (image_rotation_ == 180 || image_rotation_ == -180) {
    cv::cuda::rotate(
      gpu_frame_,
      gpu_frame_rot_,
      cv::Size(image_width_, image_height_),
      180.0,
      image_width_ - 1,
      image_height_ - 1,
      cv::InterpolationFlags::INTER_LINEAR);
    if (cinfo_manager_->isCalibrated()) {
      cv::cuda::rotate(
        gpu_rectified_frame_,
        gpu_rectified_frame_rot_,
        cv::Size(image_width_, image_height_),
        180.0,
        image_width_ - 1,
        image_height_ - 1,
        cv::InterpolationFlags::INTER_LINEAR);
    }
  }

  if (image_rotation_ != 0) {
    gpu_frame_rot_.download(frame_rot_);
    if (cinfo_manager_->isCalibrated()) {
      gpu_rectified_frame_rot_.download(frame_rect_rot_);
    }
  } else {
    gpu_frame_.download(frame_);
    if (cinfo_manager_->isCalibrated()) {
      gpu_rectified_frame_.download(rectified_frame_);
    }
  }

  return true;
#else
  cv::resize(frame_, frame_, cv::Size(image_width_, image_height_));

  if (cinfo_manager_->isCalibrated()) {
    cv::remap(
      frame_,
      rectified_frame_,
      map1_,
      map2_,
      cv::InterpolationFlags::INTER_LINEAR,
      cv::BorderTypes::BORDER_CONSTANT);
  }

  if (image_rotation_ == 90) {
    cv::rotate(frame_, frame_rot_, cv::ROTATE_90_COUNTERCLOCKWISE);
    if (cinfo_manager_->isCalibrated()) {
      cv::rotate(rectified_frame_, frame_rect_rot_, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
  } else if (image_rotation_ == -90) {
    cv::rotate(frame_, frame_rot_, cv::ROTATE_90_CLOCKWISE);
    if (cinfo_manager_->isCalibrated()) {
      cv::rotate(rectified_frame_, frame_rect_rot_, cv::ROTATE_90_CLOCKWISE);
    }
  } else if (image_rotation_ == 180 || image_rotation_ == -180) {
    cv::rotate(frame_, frame_rot_, cv::ROTATE_180);
    if (cinfo_manager_->isCalibrated()) {
      cv::rotate(rectified_frame_, frame_rect_rot_, cv::ROTATE_180);
    }
  }

  return true;
#endif
}

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr V4L2CameraDriver::frame_to_msg(cv::Mat & frame)
{
  // Allocate new image message
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant image contents
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->set__step(frame.cols * frame.elemSize());
  ros_image->set__is_bigendian(false);

  // Copy frame data
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  std::memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

/**
 * @brief Validates the camera_brightness parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool V4L2CameraDriver::validate_camera_brightness(const rclcpp::Parameter & p)
{
  if (video_cap_.isOpened() && !video_cap_.set(cv::CAP_PROP_BRIGHTNESS, p.as_double())) {
    RCLCPP_ERROR(
      this->get_logger(),
      "V4L2CameraDriver::validate_camera_brightness: Failed to set camera brightness to: %.4f",
      p.as_double());
    return false;
  }
  return true;
}

/**
 * @brief Validates the camera_exposure parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool V4L2CameraDriver::validate_camera_exposure(const rclcpp::Parameter & p)
{
  if (video_cap_.isOpened()) {
    bool success;
    if (p.as_double() == 0.0) {
      success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3.0);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::validate_camera_exposure: cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 3.0) failed");
        return false;
      }
    } else {
      success = video_cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::validate_camera_exposure: cv::VideoCapture::set(CAP_PROP_AUTO_EXPOSURE, 0.75) failed");
        return false;
      }
      success = video_cap_.set(cv::CAP_PROP_EXPOSURE, p.as_double());
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::validate_camera_exposure: cv::VideoCapture::set(CAP_PROP_EXPOSURE) failed");
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief Validates the camera_wb_temperature parameter.
 *
 * @param p Parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool V4L2CameraDriver::validate_camera_wb_temperature(const rclcpp::Parameter & p)
{
  if (video_cap_.isOpened()) {
    bool success;
    if (p.as_double() == 0.0) {
      success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 1.0);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::validate_camera_wb_temperature: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 1.0) failed");
        return false;
      }
    } else {
      success = video_cap_.set(cv::CAP_PROP_AUTO_WB, 0.0);
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::validate_camera_wb_temperature: cv::VideoCapture::set(CAP_PROP_AUTO_WB, 0.0) failed");
        return false;
      }
      success = video_cap_.set(cv::CAP_PROP_WB_TEMPERATURE, p.as_double());
      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "V4L2CameraDriver::validate_camera_wb_temperature: cv::VideoCapture::set(CAP_PROP_WB_TEMPERATURE) failed");
        return false;
      }
    }
  }
  return true;
}

} // namespace v4l2_camera_driver
