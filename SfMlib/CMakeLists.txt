cmake_minimum_required(VERSION 3.5)

find_package(OpenCV 3.1 REQUIRED COMPONENTS core calib3d features2d imgproc highgui)
find_package(Boost      REQUIRED COMPONENTS system chrono filesystem unit_test_framework program_options)
find_package(Ceres      REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
link_directories   (${Boost_LIBRARY_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

set(SFM_LIB_SOURCES Common.h                   Common.cpp
                    SfM.h                      SfM.cpp
                    FeatureUtils.h             FeatureUtils.cpp
                    StereoUtils.h         	   StereoUtils.cpp
                    BAUtil.h                   BAUtil.cpp
    )

add_library(SfMLibrary ${SFM_LIB_SOURCES} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(SfMTest sfmtest.cpp)

set(SFM_LIBS    
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
    )

target_link_libraries(SfMTest 
	SfMLibrary
	${SFM_LIBS}
	)




# cmake_minimum_required(VERSION 3.5)

# find_package(OpenCV 3.1 REQUIRED COMPONENTS core calib3d features2d imgproc highgui)

# include_directories(${OpenCV_INCLUDE_DIRS})

# add_executable(SfMTest tt.cpp)

# target_link_libraries(SfMTest 
# 	${OpenCV_LIBS}
# 	)