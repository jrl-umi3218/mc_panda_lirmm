add_robot(PandaLIRMM src/module.cpp)
target_link_libraries(PandaLIRMM PUBLIC mc_panda::panda)
target_compile_definitions(PandaLIRMM PRIVATE -DPANDA_DESCRIPTION_PATH="${PANDA_DESCRIPTION_PATH}")

install(DIRECTORY rsdf/panda2_lirmm
  DESTINATION "${PANDA_DESCRIPTION_PATH}/rsdf"
  FILES_MATCHING PATTERN "*.rsdf")
