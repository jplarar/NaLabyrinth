# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/juan/Nao/naoWorkspace/laberinto_final

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/juan/Nao/naoWorkspace/laberinto_final/build-naoqi-sdk

# Include any dependencies generated for this target.
include CMakeFiles/Laberinto_final.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Laberinto_final.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Laberinto_final.dir/flags.make

CMakeFiles/Laberinto_final.dir/main.cpp.o: CMakeFiles/Laberinto_final.dir/flags.make
CMakeFiles/Laberinto_final.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/juan/Nao/naoWorkspace/laberinto_final/build-naoqi-sdk/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Laberinto_final.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Laberinto_final.dir/main.cpp.o -c /home/juan/Nao/naoWorkspace/laberinto_final/main.cpp

CMakeFiles/Laberinto_final.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Laberinto_final.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/juan/Nao/naoWorkspace/laberinto_final/main.cpp > CMakeFiles/Laberinto_final.dir/main.cpp.i

CMakeFiles/Laberinto_final.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Laberinto_final.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/juan/Nao/naoWorkspace/laberinto_final/main.cpp -o CMakeFiles/Laberinto_final.dir/main.cpp.s

CMakeFiles/Laberinto_final.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/Laberinto_final.dir/main.cpp.o.requires

CMakeFiles/Laberinto_final.dir/main.cpp.o.provides: CMakeFiles/Laberinto_final.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Laberinto_final.dir/build.make CMakeFiles/Laberinto_final.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Laberinto_final.dir/main.cpp.o.provides

CMakeFiles/Laberinto_final.dir/main.cpp.o.provides.build: CMakeFiles/Laberinto_final.dir/main.cpp.o

# Object files for target Laberinto_final
Laberinto_final_OBJECTS = \
"CMakeFiles/Laberinto_final.dir/main.cpp.o"

# External object files for target Laberinto_final
Laberinto_final_EXTERNAL_OBJECTS =

sdk/bin/Laberinto_final: CMakeFiles/Laberinto_final.dir/main.cpp.o
sdk/bin/Laberinto_final: CMakeFiles/Laberinto_final.dir/build.make
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libalproxies.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libalproxies.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libalcommon.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libalsoap.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/librttools.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libalthread.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_signals-mt.a
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_program_options-mt.a
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libalvalue.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libtinyxml.so
sdk/bin/Laberinto_final: /usr/lib/x86_64-linux-gnu/librt.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libqi.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_filesystem-mt.a
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_thread-mt.a
sdk/bin/Laberinto_final: /usr/lib/x86_64-linux-gnu/libpthread.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libboost_system-mt.a
sdk/bin/Laberinto_final: /usr/lib/x86_64-linux-gnu/libdl.so
sdk/bin/Laberinto_final: /home/juan/Nao/devtools/naoqi-sdk-1.14.5-linux64/lib/libalerror.so
sdk/bin/Laberinto_final: /usr/local/lib/libopencv_highgui.so
sdk/bin/Laberinto_final: /usr/local/lib/libopencv_imgproc.so
sdk/bin/Laberinto_final: /usr/local/lib/libopencv_core.so
sdk/bin/Laberinto_final: CMakeFiles/Laberinto_final.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable sdk/bin/Laberinto_final"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Laberinto_final.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Laberinto_final.dir/build: sdk/bin/Laberinto_final
.PHONY : CMakeFiles/Laberinto_final.dir/build

CMakeFiles/Laberinto_final.dir/requires: CMakeFiles/Laberinto_final.dir/main.cpp.o.requires
.PHONY : CMakeFiles/Laberinto_final.dir/requires

CMakeFiles/Laberinto_final.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Laberinto_final.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Laberinto_final.dir/clean

CMakeFiles/Laberinto_final.dir/depend:
	cd /home/juan/Nao/naoWorkspace/laberinto_final/build-naoqi-sdk && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juan/Nao/naoWorkspace/laberinto_final /home/juan/Nao/naoWorkspace/laberinto_final /home/juan/Nao/naoWorkspace/laberinto_final/build-naoqi-sdk /home/juan/Nao/naoWorkspace/laberinto_final/build-naoqi-sdk /home/juan/Nao/naoWorkspace/laberinto_final/build-naoqi-sdk/CMakeFiles/Laberinto_final.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Laberinto_final.dir/depend

