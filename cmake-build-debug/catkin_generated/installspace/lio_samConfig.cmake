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


if(lio_sam_CONFIG_INCLUDED)
  return()
endif()
set(lio_sam_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(lio_sam_SOURCE_PREFIX /home/and/work/test_ws/src/SC-LIO-SAM)
  set(lio_sam_DEVEL_PREFIX /home/and/work/test_ws/src/SC-LIO-SAM/cmake-build-debug/devel)
  set(lio_sam_INSTALL_PREFIX "")
  set(lio_sam_PREFIX ${lio_sam_DEVEL_PREFIX})
else()
  set(lio_sam_SOURCE_PREFIX "")
  set(lio_sam_DEVEL_PREFIX "")
  set(lio_sam_INSTALL_PREFIX /usr/local)
  set(lio_sam_PREFIX ${lio_sam_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'lio_sam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(lio_sam_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include;/usr/local/PCL19/include/pcl-1.9;/usr/include/eigen3;/usr/include;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi;/usr/lib/x86_64-linux-gnu/openmpi/include;/usr/include/python3.8;/usr/include/hdf5/openmpi;/usr/include/jsoncpp;/usr/include/libxml2;/usr/include/tcl;/usr/include/ni;/usr/include/openni2 " STREQUAL " ")
  set(lio_sam_INCLUDE_DIRS "")
  set(_include_dirs "include;/usr/local/PCL19/include/pcl-1.9;/usr/include/eigen3;/usr/include;/usr/include/vtk-7.1;/usr/include/freetype2;/usr/include/x86_64-linux-gnu;/usr/lib/x86_64-linux-gnu/openmpi/include/openmpi;/usr/lib/x86_64-linux-gnu/openmpi/include;/usr/include/python3.8;/usr/include/hdf5/openmpi;/usr/include/jsoncpp;/usr/include/libxml2;/usr/include/tcl;/usr/include/ni;/usr/include/openni2")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Tixiao Shan <shant@mit.edu>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${lio_sam_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'lio_sam' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'lio_sam' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '\${prefix}/${idir}'.  ${_report}")
    endif()
    _list_append_unique(lio_sam_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/usr/local/PCL19/lib/libpcl_common.so;/usr/local/PCL19/lib/libpcl_kdtree.so;/usr/local/PCL19/lib/libpcl_octree.so;/usr/local/PCL19/lib/libpcl_search.so;/usr/local/PCL19/lib/libpcl_sample_consensus.so;/usr/local/PCL19/lib/libpcl_filters.so;/usr/local/PCL19/lib/libpcl_io.so;/usr/local/PCL19/lib/libpcl_features.so;/usr/local/PCL19/lib/libpcl_ml.so;/usr/local/PCL19/lib/libpcl_segmentation.so;/usr/local/PCL19/lib/libpcl_visualization.so;/usr/local/PCL19/lib/libpcl_surface.so;/usr/local/PCL19/lib/libpcl_registration.so;/usr/local/PCL19/lib/libpcl_keypoints.so;/usr/local/PCL19/lib/libpcl_tracking.so;/usr/local/PCL19/lib/libpcl_recognition.so;/usr/local/PCL19/lib/libpcl_stereo.so;/usr/local/PCL19/lib/libpcl_apps.so;/usr/local/PCL19/lib/libpcl_outofcore.so;/usr/local/PCL19/lib/libpcl_people.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;-lpthread;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;lz4;flann;flann_cpp;/usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libz.so;/usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libexpat.so;/usr/lib/x86_64-linux-gnu/libvtkDomainsChemistryOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;/usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkParallelCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelDIY2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkParallelMPI-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersPoints-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersPython-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libpython3.8.so;/usr/lib/x86_64-linux-gnu/libvtkWrappingPython38Core-7.1.so.7.1p.1;/usr/lib/libvtkWrappingTools-7.1.a;/usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkverdict-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOSQL-7.1.so.7.1p.1;sqlite3;/usr/lib/x86_64-linux-gnu/libvtkGeovisCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkproj4-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOAMR-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so;/usr/lib/x86_64-linux-gnu/libsz.so;/usr/lib/x86_64-linux-gnu/libdl.so;/usr/lib/x86_64-linux-gnu/libm.so;/usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so;/usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi_cxx.so;/usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5_hl.so;/usr/lib/x86_64-linux-gnu/libvtkIOEnSight-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOExodus-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkexoIIc-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libnetcdf_c++.so;/usr/lib/x86_64-linux-gnu/libnetcdf.so;/usr/lib/x86_64-linux-gnu/libvtkIOExport-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PSOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libgl2ps.so;/usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOMovie-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libtheoraenc.so;/usr/lib/x86_64-linux-gnu/libtheoradec.so;/usr/lib/x86_64-linux-gnu/libogg.so;/usr/lib/x86_64-linux-gnu/libvtkIOGDAL-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libjsoncpp.so;/usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOImport-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOInfovis-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libxml2.so;/usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOMINC-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOParallel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIONetCDF-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOMySQL-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOODBC-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOTecplotTable-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOVPIC-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkVPIC-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOVideo-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkxdmf2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingMath-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkImagingStencil-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkInteractionImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkLocalExample-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingExternal-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingImage-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingLICOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingSceneGraph-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeAMR-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL2-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkTestingRendering-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-7.1.so.7.1p.1;/usr/lib/x86_64-linux-gnu/libvtkWrappingJava-7.1.so.7.1p.1")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND lio_sam_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND lio_sam_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT lio_sam_NUM_DUMMY_TARGETS)
      set(lio_sam_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::lio_sam::wrapped-linker-option${lio_sam_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR lio_sam_NUM_DUMMY_TARGETS "${lio_sam_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::lio_sam::wrapped-linker-option${lio_sam_NUM_DUMMY_TARGETS}")
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
    list(APPEND lio_sam_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND lio_sam_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND lio_sam_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /usr/local/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(lio_sam_LIBRARY_DIRS ${lib_path})
      list(APPEND lio_sam_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'lio_sam'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND lio_sam_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(lio_sam_EXPORTED_TARGETS "lio_sam_generate_messages_cpp;lio_sam_generate_messages_eus;lio_sam_generate_messages_lisp;lio_sam_generate_messages_nodejs;lio_sam_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${lio_sam_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "std_msgs;nav_msgs;geometry_msgs;sensor_msgs;message_runtime;message_generation")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 lio_sam_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${lio_sam_dep}_FOUND)
      find_package(${lio_sam_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${lio_sam_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(lio_sam_INCLUDE_DIRS ${${lio_sam_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(lio_sam_LIBRARIES ${lio_sam_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${lio_sam_dep}_LIBRARIES})
  _list_append_deduplicate(lio_sam_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(lio_sam_LIBRARIES ${lio_sam_LIBRARIES})

  _list_append_unique(lio_sam_LIBRARY_DIRS ${${lio_sam_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(lio_sam_EXPORTED_TARGETS ${${lio_sam_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "lio_sam-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${lio_sam_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
