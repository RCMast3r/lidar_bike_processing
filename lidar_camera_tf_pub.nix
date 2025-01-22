{ buildRosPackage, ament-cmake-ros, ament-cmake, geometry-msgs, sensor-msgs, rclpy, launch-ros, python312Packages }:
buildRosPackage {
  pname = "ros-jazzy-lidar-camera-tf-pub";
  version = "0.0.1a";
  src = ./lidar_camera_tf_pub;

  buildType = "ament_python";
  buildInputs = [ geometry-msgs sensor-msgs python312Packages.opencv4 python312Packages.tkinter ];
  propagatedBuildInputs = [ ];
  nativeBuildInputs = [ rclpy ];

  meta = {
    description = "launch meta";
  };
}

