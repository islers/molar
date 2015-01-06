cmake_minimum_required(VERSION 2.8.3)
project(molar_headless_example)


find_package(Boost REQUIRED system filesystem)
find_package(OpenCV REQUIRED)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})


include_directories(
  ../../code_base/include/core
  ../../code_base/include/dynamic_modules
  ../../code_base/include/filters
  ../../code_base/stis/include
  ../../code_base/stis/ticpp/include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(molar_headless_example
  main.cpp
  ../../code_base/src/core/Angle.cpp
    ../../code_base/src/core/CompressedFrame.cpp
    ../../code_base/src/core/DescriptorCreator.cpp
    ../../code_base/src/core/Dynamics.cpp
    ../../code_base/src/core/ExtendedKalmanFilter.cpp
    ../../code_base/src/core/FilteredDynamics.cpp
    ../../code_base/src/core/frame.cpp
    ../../code_base/src/core/GenericObject.cpp
    ../../code_base/src/core/GOData.cpp
    ../../code_base/src/core/IPAlgorithm.cpp
    ../../code_base/src/core/objecthandler.cpp
    ../../code_base/src/core/Options.cpp
    ../../code_base/src/core/RectangleRegion.cpp
    ../../code_base/src/core/SceneHandler.cpp
    ../../code_base/src/core/sceneobject.cpp
    ../../code_base/src/core/VideoBuffer.cpp
    ../../code_base/src/dynamic_modules/DirectedRodEMA.cpp
    ../../code_base/src/dynamic_modules/FreeKalman.cpp
    ../../code_base/src/dynamic_modules/FreeMovingAverage.cpp
    ../../code_base/src/dynamic_modules/NonHoloEMA.cpp
    ../../code_base/src/dynamic_modules/NonHoloEMA3D.cpp
    ../../code_base/src/dynamic_modules/NonHoloEMA_Orth.cpp
    ../../code_base/src/dynamic_modules/NonHoloKalman2D.cpp
    ../../code_base/src/dynamic_modules/NonHoloKalman2D_Orth.cpp
    ../../code_base/src/dynamic_modules/NonHoloKalman3D.cpp
    ../../code_base/src/dynamic_modules/SimpleFreeMovement.cpp
    ../../code_base/src/dynamic_modules/StaticDynamics.cpp
    ../../code_base/src/filters/ColorRangeExpansion.cpp
    ../../code_base/src/filters/ContrastBrightnessAdjustment.cpp
    ../../code_base/stis/src/generictype.cpp
    ../../code_base/stis/ticpp/src/ticpp.cpp
    ../../code_base/stis/ticpp/src/tinystr.cpp
    ../../code_base/stis/ticpp/src/tinyxml.cpp
    ../../code_base/stis/ticpp/src/tinyxmlerror.cpp
    ../../code_base/stis/ticpp/src/tinyxmlparser.cpp
)

target_link_libraries(molar_headless_example
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS molar_headless_example DESTINATION /build)