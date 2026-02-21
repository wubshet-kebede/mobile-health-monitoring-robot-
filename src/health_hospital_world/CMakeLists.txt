cmake_minimum_required(VERSION 3.8)
project(health_hospital_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# This replaces the AWS Python script
# It runs your shell script to download all 40+ models automatically
set(MODEL_LIST
  XRayMachine IVStand BloodPressureMonitor BPCart BMWCart 
  CGMClassic StorageRack Chair InstrumentCart1 Scrubs 
  PatientWheelChair WhiteChipChair TrolleyBed SurgicalTrolley 
  PotatoChipChair VisitorKidSit FemaleVisitorSit AdjTable 
  MopCart3 MaleVisitorSit Drawer OfficeChairBlack ElderLadyPatient 
  ElderMalePatient InstrumentCart2 MetalCabinet BedTable BedsideTable 
  AnesthesiaMachine TrolleyBedPatient Shower SurgicalTrolleyMed 
  StorageRackCovered KitchenSink Toilet VendingMachine ParkingTrolleyMin 
  PatientFSit MaleVisitorOnPhone FemaleVisitor MalePatientBed 
  StorageRackCoverOpen ParkingTrolleyMax 
)

set(MISSING_MODELS FALSE)
foreach(MODEL ${MODEL_LIST})
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models/${MODEL})
    set(MISSING_MODELS TRUE)
  endif()
endforeach()

if(MISSING_MODELS)
  message(STATUS "Some models missing, downloading hospital models...")
  execute_process(
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/download_models.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
else()
  message(STATUS "All models already downloaded, skipping download.")
endif()

# Install everything so ROS 2 Jazzy can find it
install(DIRECTORY launch models worlds
    DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

