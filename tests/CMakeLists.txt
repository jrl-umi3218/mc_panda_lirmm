add_executable(test_module test_module.cpp)
target_link_libraries(test_module PUBLIC mc_rtc::mc_rbdyn mc_panda_lirmm::panda_lirmm)
add_test(NAME test_module COMMAND test_module "$<TARGET_FILE_DIR:mc_panda_lirmm>")
