# Install calibrated model for panda2lirmm
# It shares meshes/rsdf/convex with the panda model, but has a different optimized urdf
# So only the urdf path differs

set(CALIBRATED_PANDA_LIRMM_URDF_PATH "${CMAKE_INSTALL_PREFIX}/share/mc_panda_lirmm/calibrated_urdf")
# URDF_DESTINATION points to PANDA_DESCRIPTION_PATH, exported from mc_panda
# This is done on purpose so that meshes in the calibrated urdfs
set(URDF_DESTINATION "${FR1_DESCRIPTION_PATH}/")
set(URDF_OUT "${CMAKE_BINARY_DIR}/calibrated_urdf/panda2lirmm/fr1/urdf/panda_default.urdf")
configure_file(panda2lirmm/fr1/panda_default.in.urdf "${URDF_OUT}" @ONLY)
install(FILES "${URDF_OUT}" DESTINATION "${CALIBRATED_PANDA_LIRMM_URDF_PATH}/panda2lirmm/fr1/urdf/")
