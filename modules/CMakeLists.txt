# Panda robot module library
set(panda_lirmm_HDR
  include/mc_panda_lirmm/panda_lirmm.h
  "${CMAKE_CURRENT_SOURCE_DIR}/include/mc_panda_lirmm/panda_lirmm.h"
)

set(panda_lirmm_SRC
  src/panda_lirmm.cpp
)
configure_file(include/mc_panda_lirmm/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/mc_panda_lirmm/config.h")

add_library(mc_panda_lirmm SHARED ${panda_lirmm_SRC} ${panda_lirmm_HDR})
add_library(mc_panda_lirmm::panda_lirmm ALIAS mc_panda_lirmm)
set_target_properties(mc_panda_lirmm PROPERTIES EXPORT_NAME panda_lirmm)
set_target_properties(mc_panda_lirmm PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
target_link_libraries(mc_panda_lirmm PUBLIC mc_panda::panda)
target_include_directories(mc_panda_lirmm PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)
target_include_directories(mc_panda_lirmm PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_definitions(mc_panda_lirmm PRIVATE -DMC_PANDA_EXPORTS)
install(FILES ${panda_lirmm_HDR} DESTINATION include/mc_panda_lirmm)

install(TARGETS mc_panda_lirmm
  EXPORT mc_panda_lirmmTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(EXPORT mc_panda_lirmmTargets
  FILE mc_panda_lirmmTargets.cmake
  NAMESPACE mc_panda_lirmm::
  DESTINATION lib/cmake/mc_panda_lirmm)

add_robot(PandaLIRMM src/module.cpp)
target_link_libraries(PandaLIRMM PUBLIC mc_panda_lirmm::panda_lirmm)
target_compile_definitions(PandaLIRMM PRIVATE -DPANDA_DESCRIPTION_PATH="${PANDA_DESCRIPTION_PATH}")

install(DIRECTORY rsdf/panda2_lirmm
  DESTINATION "${PANDA_DESCRIPTION_PATH}/rsdf"
  FILES_MATCHING PATTERN "*.rsdf")
