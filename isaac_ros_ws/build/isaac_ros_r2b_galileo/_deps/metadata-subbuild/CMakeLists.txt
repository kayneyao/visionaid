# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(metadata-populate NONE)



include(ExternalProject)
ExternalProject_Add(metadata-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://api.ngc.nvidia.com/v2/resources/org/nvidia/team/isaac/r2bdataset2024/1/files?redirect=true&path=r2b_galileo/metadata.yaml" "DOWNLOAD_DIR" "/home/sophie/visionaid/isaac_ros_ws/build/isaac_ros_r2b_galileo/data/r2b_galileo/" DOWNLOAD_NO_EXTRACT YES
                    SOURCE_DIR          "/home/sophie/visionaid/isaac_ros_ws/build/isaac_ros_r2b_galileo/_deps/metadata-src"
                    BINARY_DIR          "/home/sophie/visionaid/isaac_ros_ws/build/isaac_ros_r2b_galileo/_deps/metadata-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


ExternalProject_Get_Property(metadata-populate DOWNLOADED_FILE)
get_filename_component(dlFileName "${DOWNLOADED_FILE}" NAME)

ExternalProject_Add_Step(metadata-populate copyfile
  COMMAND    "/usr/bin/cmake" -E copy_if_different
             "<DOWNLOADED_FILE>" "/home/sophie/visionaid/isaac_ros_ws/build/isaac_ros_r2b_galileo/_deps/metadata-src"
  DEPENDEES  patch
  DEPENDERS  configure
  BYPRODUCTS "/home/sophie/visionaid/isaac_ros_ws/build/isaac_ros_r2b_galileo/_deps/metadata-src/${dlFileName}"
  COMMENT    "Copying file to SOURCE_DIR"
)

