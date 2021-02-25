# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(sspp_CONFIG_INCLUDED)
  return()
endif()
set(sspp_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(sspp_SOURCE_PREFIX /home/seg/git_catkin_ws/src/asscpp/sspp)
  set(sspp_DEVEL_PREFIX /home/seg/git_catkin_ws/src/asscpp/sspp/cmake-build-debug/devel)
  set(sspp_INSTALL_PREFIX "")
  set(sspp_PREFIX ${sspp_DEVEL_PREFIX})
else()
  set(sspp_SOURCE_PREFIX "")
  set(sspp_DEVEL_PREFIX "")
  set(sspp_INSTALL_PREFIX /usr/local)
  set(sspp_PREFIX ${sspp_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'sspp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(sspp_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/seg/git_catkin_ws/src/asscpp/sspp/cmake-build-debug/devel/include;/home/seg/git_catkin_ws/src/asscpp/sspp/include " STREQUAL " ")
  set(sspp_INCLUDE_DIRS "")
  set(_include_dirs "/home/seg/git_catkin_ws/src/asscpp/sspp/cmake-build-debug/devel/include;/home/seg/git_catkin_ws/src/asscpp/sspp/include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT "https://github.com/kuri-kustar/sspp/wiki " STREQUAL " ")
    set(_report "Check the website 'https://github.com/kuri-kustar/sspp/wiki' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Randa Almadhoun <randa.almadhoun@gmail.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${sspp_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'sspp' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'sspp' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/seg/git_catkin_ws/src/asscpp/sspp/${idir}'.  ${_report}")
    endif()
    _list_append_unique(sspp_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/opt/ros/noetic/lib/libpcl_ros_filter.so;/opt/ros/noetic/lib/libpcl_ros_tf.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;-lpthread;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so;/usr/lib/x86_64-linux-gnu/libpcl_search.so;/usr/lib/x86_64-linux-gnu/libpcl_features.so;/usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so;/usr/lib/x86_64-linux-gnu/libpcl_filters.so;/usr/lib/x86_64-linux-gnu/libpcl_ml.so;/usr/lib/x86_64-linux-gnu/libpcl_segmentation.so;/usr/lib/x86_64-linux-gnu/libpcl_surface.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/x86_64-linux-gnu/libflann_cpp.so;/opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so;/opt/ros/noetic/lib/libnodeletlib.so;/opt/ros/noetic/lib/libbondcpp.so;/usr/lib/x86_64-linux-gnu/libuuid.so;/usr/lib/x86_64-linux-gnu/libpcl_common.so;/usr/lib/x86_64-linux-gnu/libpcl_octree.so;/usr/lib/x86_64-linux-gnu/libpcl_io.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libexpat.so;/usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1;/opt/ros/noetic/lib/librosbag.so;/opt/ros/noetic/lib/librosbag_storage.so;/opt/ros/noetic/lib/libclass_loader.so;/usr/lib/x86_64-linux-gnu/libPocoFoundation.so;/usr/lib/x86_64-linux-gnu/libdl.so;/opt/ros/noetic/lib/libroslib.so;/opt/ros/noetic/lib/librospack.so;/usr/lib/x86_64-linux-gnu/libpython3.8.so;/usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0;/usr/lib/x86_64-linux-gnu/libtinyxml2.so;/opt/ros/noetic/lib/libroslz4.so;/usr/lib/x86_64-linux-gnu/liblz4.so;/opt/ros/noetic/lib/libtopic_tools.so;/opt/ros/noetic/lib/libtf_conversions.so;/opt/ros/noetic/lib/libkdl_conversions.so;/opt/ros/noetic/lib/libtf.so;/opt/ros/noetic/lib/libeigen_conversions.so;/usr/lib/liborocos-kdl.so;/opt/ros/noetic/lib/librviz_visual_tools.so;/opt/ros/noetic/lib/librviz_visual_tools_gui.so;/opt/ros/noetic/lib/librviz_visual_tools_remote_control.so;/opt/ros/noetic/lib/librviz_visual_tools_imarker_simple.so;/opt/ros/noetic/lib/libinteractive_markers.so;/opt/ros/noetic/lib/libtf2_ros.so;/opt/ros/noetic/lib/libactionlib.so;/opt/ros/noetic/lib/libmessage_filters.so;/opt/ros/noetic/lib/libroscpp.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0;/opt/ros/noetic/lib/librosconsole.so;/opt/ros/noetic/lib/librosconsole_log4cxx.so;/opt/ros/noetic/lib/librosconsole_backend_interface.so;/usr/lib/x86_64-linux-gnu/liblog4cxx.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0;/opt/ros/noetic/lib/libxmlrpcpp.so;/opt/ros/noetic/lib/libtf2.so;/opt/ros/noetic/lib/libroscpp_serialization.so;/opt/ros/noetic/lib/librostime.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0;/opt/ros/noetic/lib/libcpp_common.so;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0;/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4;/opt/ros/noetic/lib/liboctomap.so;/opt/ros/noetic/lib/liboctomath.so;SSPathPlanner")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND sspp_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND sspp_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT sspp_NUM_DUMMY_TARGETS)
      set(sspp_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::sspp::wrapped-linker-option${sspp_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR sspp_NUM_DUMMY_TARGETS "${sspp_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::sspp::wrapped-linker-option${sspp_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND sspp_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND sspp_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND sspp_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/seg/git_catkin_ws/src/asscpp/sspp/cmake-build-debug/devel/lib;/home/seg/git_catkin_ws/devel/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(sspp_LIBRARY_DIRS ${lib_path})
      list(APPEND sspp_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'sspp'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND sspp_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(sspp_EXPORTED_TARGETS "sspp_generate_messages_cpp;sspp_generate_messages_eus;sspp_generate_messages_lisp;sspp_generate_messages_nodejs;sspp_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${sspp_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;pcl_ros")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 sspp_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${sspp_dep}_FOUND)
      find_package(${sspp_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${sspp_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(sspp_INCLUDE_DIRS ${${sspp_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(sspp_LIBRARIES ${sspp_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${sspp_dep}_LIBRARIES})
  _list_append_deduplicate(sspp_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(sspp_LIBRARIES ${sspp_LIBRARIES})

  _list_append_unique(sspp_LIBRARY_DIRS ${${sspp_dep}_LIBRARY_DIRS})
  list(APPEND sspp_EXPORTED_TARGETS ${${sspp_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "sspp-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${sspp_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
