set(CALIBRATED_PANDA_LIRMM_URDF_PATH "${CMAKE_INSTALL_PREFIX}/share/mc_panda_lirmm/calibrated_urdf")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/config.in.h ${CMAKE_BINARY_DIR}/include/config.h)

add_robot(PandaLIRMM src/module.cpp)
target_link_libraries(PandaLIRMM PUBLIC mc_panda::panda)
target_include_directories(PandaLIRMM PUBLIC $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>)
