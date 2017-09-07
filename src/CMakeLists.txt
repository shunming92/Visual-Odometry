

set(OpenCV_DIR "/home/li/software/opencv-3.1.0/build")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(G2O REQUIRED)
find_package( CSparse REQUIRED )
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

# include_directories( ${PROJECT_SOURCE_DIR}/include )

include_directories(
${EIGEN3_INCLUDE_DIR}
# ${CERES_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
)

message(STATUS "########## Third Package Address ##########")
message(STATUS "Opencv: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "Eigen3: " ${EIGEN3_INCLUDE_DIR})
message(STATUS "G2O: " ${G2O_INCLUDE_DIRS})
message(STATUS "###########################################")

add_library (${PROJECT_NAME} SHARED
LoadData.cpp
System.cpp
imagePorcess/ImageProcessor.cpp
imagePorcess/Rectifier.cpp
imagePorcess/OrbExtractor.cpp
imagePorcess/Matcher.cpp
track/Track.cpp
track/Frame.cpp
track/optimizor.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${CERES_LIBRARIES}
${CSPARSE_LIBRARY}
g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)
add_executable( main main.cpp)
target_link_libraries(main ${PROJECT_NAME})
