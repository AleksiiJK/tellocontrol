# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/eeavir/drone_ws/src/drone_racing_ros2/tello_ros/tello_description

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eeavir/drone_ws/build/tello_description

# Utility rule file for generate_urdf.

# Include the progress variables for this target.
include CMakeFiles/generate_urdf.dir/progress.make

CMakeFiles/generate_urdf: urdf/tello.urdf


urdf/tello.urdf: /home/eeavir/drone_ws/src/drone_racing_ros2/tello_ros/tello_description/urdf/tello.xml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/eeavir/drone_ws/build/tello_description/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generate /home/eeavir/drone_ws/build/tello_description/urdf/tello.urdf"
	/usr/bin/python3 /home/eeavir/drone_ws/src/drone_racing_ros2/tello_ros/tello_description/src/replace.py /home/eeavir/drone_ws/src/drone_racing_ros2/tello_ros/tello_description/urdf/tello.xml suffix= topic_ns=solo > /home/eeavir/drone_ws/build/tello_description/urdf/tello.urdf

generate_urdf: CMakeFiles/generate_urdf
generate_urdf: urdf/tello.urdf
generate_urdf: CMakeFiles/generate_urdf.dir/build.make

.PHONY : generate_urdf

# Rule to build all files generated by this target.
CMakeFiles/generate_urdf.dir/build: generate_urdf

.PHONY : CMakeFiles/generate_urdf.dir/build

CMakeFiles/generate_urdf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/generate_urdf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/generate_urdf.dir/clean

CMakeFiles/generate_urdf.dir/depend:
	cd /home/eeavir/drone_ws/build/tello_description && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eeavir/drone_ws/src/drone_racing_ros2/tello_ros/tello_description /home/eeavir/drone_ws/src/drone_racing_ros2/tello_ros/tello_description /home/eeavir/drone_ws/build/tello_description /home/eeavir/drone_ws/build/tello_description /home/eeavir/drone_ws/build/tello_description/CMakeFiles/generate_urdf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/generate_urdf.dir/depend

