# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/contour/ws_catkin_elephant/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/contour/ws_catkin_elephant/build

# Include any dependencies generated for this target.
include elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/compiler_depend.make

# Include the progress variables for this target.
include elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/progress.make

# Include the compile flags for this target's objects.
include elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/flags.make

elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o: elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/flags.make
elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o: /home/contour/ws_catkin_elephant/src/elephant/elerobot_socket/src/elerobot_tcpsocket_client.cpp
elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o: elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/contour/ws_catkin_elephant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o"
	cd /home/contour/ws_catkin_elephant/build/elephant/elerobot_socket && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o -MF CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o.d -o CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o -c /home/contour/ws_catkin_elephant/src/elephant/elerobot_socket/src/elerobot_tcpsocket_client.cpp

elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.i"
	cd /home/contour/ws_catkin_elephant/build/elephant/elerobot_socket && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/contour/ws_catkin_elephant/src/elephant/elerobot_socket/src/elerobot_tcpsocket_client.cpp > CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.i

elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.s"
	cd /home/contour/ws_catkin_elephant/build/elephant/elerobot_socket && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/contour/ws_catkin_elephant/src/elephant/elerobot_socket/src/elerobot_tcpsocket_client.cpp -o CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.s

# Object files for target elerobot_tcpsocket_client
elerobot_tcpsocket_client_OBJECTS = \
"CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o"

# External object files for target elerobot_tcpsocket_client
elerobot_tcpsocket_client_EXTERNAL_OBJECTS =

/home/contour/ws_catkin_elephant/devel/lib/libelerobot_tcpsocket_client.so: elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/src/elerobot_tcpsocket_client.cpp.o
/home/contour/ws_catkin_elephant/devel/lib/libelerobot_tcpsocket_client.so: elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/build.make
/home/contour/ws_catkin_elephant/devel/lib/libelerobot_tcpsocket_client.so: elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/contour/ws_catkin_elephant/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/contour/ws_catkin_elephant/devel/lib/libelerobot_tcpsocket_client.so"
	cd /home/contour/ws_catkin_elephant/build/elephant/elerobot_socket && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/elerobot_tcpsocket_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/build: /home/contour/ws_catkin_elephant/devel/lib/libelerobot_tcpsocket_client.so
.PHONY : elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/build

elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/clean:
	cd /home/contour/ws_catkin_elephant/build/elephant/elerobot_socket && $(CMAKE_COMMAND) -P CMakeFiles/elerobot_tcpsocket_client.dir/cmake_clean.cmake
.PHONY : elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/clean

elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/depend:
	cd /home/contour/ws_catkin_elephant/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/contour/ws_catkin_elephant/src /home/contour/ws_catkin_elephant/src/elephant/elerobot_socket /home/contour/ws_catkin_elephant/build /home/contour/ws_catkin_elephant/build/elephant/elerobot_socket /home/contour/ws_catkin_elephant/build/elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : elephant/elerobot_socket/CMakeFiles/elerobot_tcpsocket_client.dir/depend
