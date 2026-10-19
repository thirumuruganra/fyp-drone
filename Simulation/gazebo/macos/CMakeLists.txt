cmake_minimum_required(VERSION 3.16)
project(FypGazebo LANGUAGES CXX OBJCXX)
find_package(gz-sim8 REQUIRED COMPONENTS gui)
add_executable(FypGazebo MACOSX_BUNDLE main.mm)
target_compile_features(FypGazebo PRIVATE cxx_std_17)
target_link_libraries(FypGazebo PRIVATE gz-sim8::gui "-framework Cocoa")
set_target_properties(FypGazebo PROPERTIES
  OUTPUT_NAME "FYP Gazebo"
  MACOSX_BUNDLE_GUI_IDENTIFIER "org.fypb28.gazebo"
  MACOSX_BUNDLE_BUNDLE_NAME "FYP Gazebo"
  MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in")
