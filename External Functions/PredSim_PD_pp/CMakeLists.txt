set(TARGET_NAME PredSim_PD_pp)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
target_link_libraries(${TARGET_NAME} osimSimulation)
set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER "External_Functions")
		