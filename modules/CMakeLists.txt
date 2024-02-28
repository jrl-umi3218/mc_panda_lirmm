configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")
add_robot(PandaLIRMM module.cpp module.h)
target_link_libraries(PandaLIRMM PUBLIC mc_panda::panda)
target_include_directories(PandaLIRMM PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

install(DIRECTORY rsdf/panda2_lirmm
  DESTINATION "${PANDA_DESCRIPTION_PATH}/rsdf"
  FILES_MATCHING PATTERN "*.rsdf")
