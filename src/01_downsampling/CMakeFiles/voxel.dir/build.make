# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src/01_downsampling

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/src/01_downsampling

# Include any dependencies generated for this target.
include CMakeFiles/voxel.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/voxel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/voxel.dir/flags.make

CMakeFiles/voxel.dir/src/voxelization.cpp.o: CMakeFiles/voxel.dir/flags.make
CMakeFiles/voxel.dir/src/voxelization.cpp.o: src/voxelization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/01_downsampling/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/voxel.dir/src/voxelization.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/voxel.dir/src/voxelization.cpp.o -c /home/ubuntu/catkin_ws/src/01_downsampling/src/voxelization.cpp

CMakeFiles/voxel.dir/src/voxelization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/voxel.dir/src/voxelization.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/01_downsampling/src/voxelization.cpp > CMakeFiles/voxel.dir/src/voxelization.cpp.i

CMakeFiles/voxel.dir/src/voxelization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/voxel.dir/src/voxelization.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/01_downsampling/src/voxelization.cpp -o CMakeFiles/voxel.dir/src/voxelization.cpp.s

CMakeFiles/voxel.dir/src/voxelization.cpp.o.requires:

.PHONY : CMakeFiles/voxel.dir/src/voxelization.cpp.o.requires

CMakeFiles/voxel.dir/src/voxelization.cpp.o.provides: CMakeFiles/voxel.dir/src/voxelization.cpp.o.requires
	$(MAKE) -f CMakeFiles/voxel.dir/build.make CMakeFiles/voxel.dir/src/voxelization.cpp.o.provides.build
.PHONY : CMakeFiles/voxel.dir/src/voxelization.cpp.o.provides

CMakeFiles/voxel.dir/src/voxelization.cpp.o.provides.build: CMakeFiles/voxel.dir/src/voxelization.cpp.o


# Object files for target voxel
voxel_OBJECTS = \
"CMakeFiles/voxel.dir/src/voxelization.cpp.o"

# External object files for target voxel
voxel_EXTERNAL_OBJECTS =

voxel: CMakeFiles/voxel.dir/src/voxelization.cpp.o
voxel: CMakeFiles/voxel.dir/build.make
voxel: /usr/lib/x86_64-linux-gnu/libboost_system.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_thread.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_regex.so
voxel: /usr/lib/x86_64-linux-gnu/libpthread.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_common.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
voxel: /usr/lib/libOpenNI.so
voxel: /usr/lib/libOpenNI2.so
voxel: /usr/lib/x86_64-linux-gnu/libfreetype.so
voxel: /usr/lib/x86_64-linux-gnu/libz.so
voxel: /usr/lib/x86_64-linux-gnu/libexpat.so
voxel: /usr/lib/x86_64-linux-gnu/libpython2.7.so
voxel: /usr/lib/libvtkWrappingTools-6.3.a
voxel: /usr/lib/x86_64-linux-gnu/libjpeg.so
voxel: /usr/lib/x86_64-linux-gnu/libpng.so
voxel: /usr/lib/x86_64-linux-gnu/libtiff.so
voxel: /usr/lib/x86_64-linux-gnu/libproj.so
voxel: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
voxel: /usr/lib/x86_64-linux-gnu/libsz.so
voxel: /usr/lib/x86_64-linux-gnu/libdl.so
voxel: /usr/lib/x86_64-linux-gnu/libm.so
voxel: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
voxel: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
voxel: /usr/lib/x86_64-linux-gnu/libnetcdf.so
voxel: /usr/lib/x86_64-linux-gnu/libgl2ps.so
voxel: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
voxel: /usr/lib/x86_64-linux-gnu/libtheoradec.so
voxel: /usr/lib/x86_64-linux-gnu/libogg.so
voxel: /usr/lib/x86_64-linux-gnu/libxml2.so
voxel: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_io.so
voxel: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
voxel: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_search.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_features.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
voxel: /usr/lib/x86_64-linux-gnu/libqhull.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_people.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_system.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_thread.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
voxel: /usr/lib/x86_64-linux-gnu/libboost_regex.so
voxel: /usr/lib/x86_64-linux-gnu/libpthread.so
voxel: /usr/lib/x86_64-linux-gnu/libqhull.so
voxel: /usr/lib/libOpenNI.so
voxel: /usr/lib/libOpenNI2.so
voxel: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
voxel: /usr/lib/x86_64-linux-gnu/libfreetype.so
voxel: /usr/lib/x86_64-linux-gnu/libz.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libexpat.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libpython2.7.so
voxel: /usr/lib/libvtkWrappingTools-6.3.a
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkverdict-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libjpeg.so
voxel: /usr/lib/x86_64-linux-gnu/libpng.so
voxel: /usr/lib/x86_64-linux-gnu/libtiff.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libproj.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
voxel: /usr/lib/x86_64-linux-gnu/libsz.so
voxel: /usr/lib/x86_64-linux-gnu/libdl.so
voxel: /usr/lib/x86_64-linux-gnu/libm.so
voxel: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
voxel: /usr/lib/x86_64-linux-gnu/libnetcdf.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOExport-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libgl2ps.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
voxel: /usr/lib/x86_64-linux-gnu/libtheoradec.so
voxel: /usr/lib/x86_64-linux-gnu/libogg.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOImport-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libxml2.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkVPIC-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingExternal-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeAMR-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libpcl_common.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_io.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_search.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_features.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
voxel: /usr/lib/x86_64-linux-gnu/libpcl_people.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
voxel: /usr/lib/x86_64-linux-gnu/libtheoradec.so
voxel: /usr/lib/x86_64-linux-gnu/libogg.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
voxel: /usr/lib/x86_64-linux-gnu/libnetcdf.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libxml2.so
voxel: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
voxel: /usr/lib/x86_64-linux-gnu/libsz.so
voxel: /usr/lib/x86_64-linux-gnu/libdl.so
voxel: /usr/lib/x86_64-linux-gnu/libm.so
voxel: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libpython2.7.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
voxel: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
voxel: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libGLU.so
voxel: /usr/lib/x86_64-linux-gnu/libSM.so
voxel: /usr/lib/x86_64-linux-gnu/libICE.so
voxel: /usr/lib/x86_64-linux-gnu/libX11.so
voxel: /usr/lib/x86_64-linux-gnu/libXext.so
voxel: /usr/lib/x86_64-linux-gnu/libXt.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOXML-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libz.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libfreetype.so
voxel: /usr/lib/x86_64-linux-gnu/libGL.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libvtksys-6.3.so.6.3.0
voxel: /usr/lib/x86_64-linux-gnu/libproj.so
voxel: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0
voxel: CMakeFiles/voxel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/catkin_ws/src/01_downsampling/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable voxel"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/voxel.dir/build: voxel

.PHONY : CMakeFiles/voxel.dir/build

CMakeFiles/voxel.dir/requires: CMakeFiles/voxel.dir/src/voxelization.cpp.o.requires

.PHONY : CMakeFiles/voxel.dir/requires

CMakeFiles/voxel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/voxel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/voxel.dir/clean

CMakeFiles/voxel.dir/depend:
	cd /home/ubuntu/catkin_ws/src/01_downsampling && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src/01_downsampling /home/ubuntu/catkin_ws/src/01_downsampling /home/ubuntu/catkin_ws/src/01_downsampling /home/ubuntu/catkin_ws/src/01_downsampling /home/ubuntu/catkin_ws/src/01_downsampling/CMakeFiles/voxel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/voxel.dir/depend

