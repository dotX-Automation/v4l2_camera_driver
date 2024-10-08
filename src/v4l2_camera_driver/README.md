# v4l2_camera_driver

Simple ROS 2 driver node for USB cameras compatible with the `Video4Linux` APIs. Based on `image_transport`, `camera_calibration` and `OpenCV`.

## Features

- `CameraInfo` topic.
- `CompressedImage` topics for both color and rectified-color images.
- `Image` topics for both color and rectified-color images.
- `Theora` resettable publisher based on the `TheoraWrappers` library.
- Hardware enable service, based on `std_srvs/srv/SetBool`.
- Supports namespace and node name remappings, in order to run different cameras with multiple instances of the node.
- ROS 2 component compilation and installation.
- Optimized memory handling.
- Supports Nvidia CUDA hardware, the OpenCV GPU module, and the Nvidia VPI API.
- High-resolution, thread-based camera sampling.
- Offers both reliable and best-effort QoS profiles, configurable via node parameters.

## Usage

The code compiles to both a standalone application executable and a ROS 2 component, and both can be run easily. There is also a launch file for the standalone application.

The CMake configuration automatically detects if a compatible CUDA Toolkit installation is available, and if OpenCV has been built with CUDA support. In such case, parts of the code that perform processing such as rectification are replaced with `cv::cuda` API calls. For this to work, OpenCV must have been built from source with CUDA support, which requires a working installation of the CUDA Toolkit and a compatible Nvidia GPU.

Once the node is started, the video capture device will be disabled by default. To toggle it, send a request on the `~/enable_camera` service specifying either `True` or `False` in the `data` field.

You can use `RViz` to display the frames being streamed:

- topic `Reliability Policy` must be set to what the corresponding node parameter has been set to;
- `Fixed Frame` in `Global Options` must be set appropriately, and a transform from the camera frame to the fixed frame must be available.

### Build options

The build process is mostly automatic, and CMake should detect the best configuration for the machine it is running on, including containerized environments. However, some options can be specified manually:

- `NO_CUDA`: disables CUDA code paths, even if a compatible installation is available (default: `OFF`).
- `VPI`: enables Nvidia VPI code paths (default: `OFF`).

Build options can be passed to CMake via `colcon`:

```bash
colcon build --ament-cmake-args "-DNO_CUDA=ON" ...
```

The VPI pipeline goes from image acquisition to resize, rectification, rotation, and encoding back in host memory. There is a single stream that attempts to run frame preparation in the CUDA backend, and remappings (rectification, rotation) on the VIC if available (*e.g.*, on Jetson devices). The stream is synchronized before encoding and publishing.

### Node parameters

Configuration files for node parameters can be found in `config`, with some standard default settings. They can be customized, or overridden from command line or launch files.

- `autostart`: starts the camera driver on node initialization.
- `camera_brightness`: camera brightness level (hardware-dependent).
- `camera_calibration_file`: camera calibration YAML file URL.
- `camera_device_file`: camera device file name.
- `camera_exposure`: camera exposure time (hardware-dependent).
- `camera_fps`: camera capture rate, defaults to `20`.
- `camera_frame_id`: id of the camera link, defaults to `camera_link`.
- `camera_id`: ID of the video capture device to open, alternative to `camera_device_file`.
- `camera_name`: camera name in the configuration file.
- `camera_wb_temperature`: white balance temperature (hardware-dependent).
- `image_height`: image height, defaults to `480`.
- `image_rotation`: image rotation angle \[deg\], defaults to `0` and must be a multiple of `90°`.
- `image_width`: image width, defaults to `640`.
- `publisher_base_topic_name`: base transmission topic name for `image_transport` publishers.
- `publisher_best_effort_qos`: enables unreliable but faster transmissions.
- `publisher_depth`: depth of the image publisher queue.

Keep in mind that:

- Hardware-dependent parameters are particularly tricky: they might not be supported, have unusual or even completely different ranges, and require some black magic to be correctly set up. What you see in this code was done to work with some cameras we had at the time, so be prepared to change many things if you want to act on camera hardware settings.
- Image rotation is intended as the last stage in the post-processing pipeline applied by this node, so camera parameters and image resolution (width, height) must be provided for the camera in its default orientation.

See the [`params.yaml`](src/v4l2_camera_driver/src/v4l2_camera_driver/params.yaml) file for more information.

### Camera calibration

The necessary parameters and camera intrinsics can be acquired from a standard calibration procedure. You can write your own routine for this, e.g. with `OpenCV`, or use the `camera_calibration cameracalibrator` tool as documented [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html).

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
