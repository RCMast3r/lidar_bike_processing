{
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    # ouster-ros-src.url = "github:ouster-lidar/ouster-ros/ros2?submodules=1";
    # ouster-ros-src = {
    # url = "git+https://github.com/ouster-lidar/ouster-ros.git?ref=refs/heads/ros2&?submodules=1";
    # # branch = "ros2";
    # flake = false;
    # };
    # ouster-ros-src.flake = false;
    # ouster-ros-src.submodules = true;
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default my-ros-overlay ];
        };
        my_overlay = final: prev: {
          lidar-camera-tf-pub = final.callPackage ./lidar_camera_tf_pub.nix { };
        };

        my-ros-overlay = final: prev: {
          rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope my_overlay; };
        };
      in {
        devShells.default = pkgs.mkShell {
          shellHook = ''
            sudo sysctl -w net.core.rmem_max=2147483647
          '';
          name = "lidar-bike-env";
          RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST";
          RMW_CONNEXT_PUBLICATION_MODE="ASYNCHRONOUS";
          CYCLONEDDS_URI="file://config/ddsconfig.xml";
          
          packages = [
            pkgs.colcon
            pkgs.python311Packages.tkinter
            # ... other non-ROS packages
            (with pkgs.rosPackages.jazzy; buildEnv {
                paths = [
                    ros-core
                    ros-base
                    foxglove-bridge
                    rosbag2-storage-mcap
                    (ouster-ros.overrideAttrs (finalAttrs: previousAttrs: {
                      buildType = "ament_cmake";
                      buildInputs = with pkgs; [ ament-cmake eigen pcl rosidl-default-generators tf2-eigen ];
                      checkInputs = [ gtest ];
                      propagatedBuildInputs = with pkgs; [ tf2-eigen curl geometry-msgs jsoncpp launch launch-ros libtins ouster-sensor-msgs pcl-conversions pcl-ros rclcpp rclcpp-components rclcpp-lifecycle rosidl-default-runtime sensor-msgs spdlog std-msgs std-srvs tf2-ros ];
                      nativeBuildInputs = [ ament-cmake rosidl-default-generators ];
                      src = pkgs.fetchurl {
                        url = "https://github.com/ros2-gbp/ouster-ros-release/archive/release/jazzy/ouster_ros/0.13.2.tar.gz";
                        name = "0.13.2.tar.gz";
                        sha256 = "sha256-TEO7xqCYxkDCcXejx0qV/sSL1VQccntUI5+q2KtjOJA=";
                      };
                    }))
                    rmw-cyclonedds-cpp
                    ublox
                    # imu-gps-driver
                    nmea-navsat-driver
                    lidar-camera-tf-pub 
                    (usb-cam.overrideAttrs (finalAttrs: previousAttrs: {
                      propagatedBuildInputs = with pkgs; [ builtin-interfaces camera-info-manager cv-bridge ffmpeg_4 image-transport image-transport-plugins rclcpp rclcpp-components rosidl-default-runtime sensor-msgs std-msgs std-srvs v4l-utils ];
                      nativeBuildInputs = previousAttrs.nativeBuildInputs ++ [ pkgs.pkg-config ];
                    }))
                ];
            })
          ];
        };

        packages = pkgs;
        legacyPackages =
              import nixpkgs {
                inherit system;
                overlays = [
                  nix-ros-overlay.overlays.default
                  my-ros-overlay
                ];
              };
      });
  
}
