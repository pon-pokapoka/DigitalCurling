
# --- Box2D ---
set(DCSIM_FCV1Table_BOX2D_VERSION "v2.4.1")
include(FetchContent)
FetchContent_Declare(
    box2d
    GIT_REPOSITORY "https://github.com/erincatto/box2d.git"
    GIT_TAG        ${DCSIM_FCV1Table_BOX2D_VERSION}
    PATCH_COMMAND  ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/patch_box2d.cmake"
    SYSTEM
)
set(BOX2D_BUILD_TESTBED OFF)
set(BOX2D_BUILD_UNIT_TESTS OFF)
FetchContent_MakeAvailable(box2d)

# --- Build plugin object ---
add_library(digitalcurling_simulator_fcv1table_obj OBJECT
    "./simulator_fcv1table.cpp"
    "./simulator_fcv1table_factory.cpp"
    "./simulator_fcv1table_storage.cpp"
    "./free_motion_table.cpp"
)
target_include_directories(digitalcurling_simulator_fcv1table_obj
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(digitalcurling_simulator_fcv1table_obj
    PUBLIC  digitalcurling::plugin_api
    PUBLIC  box2d  # the tests include the simulator's header
)

# --- Build plugin ---
add_library(digitalcurling_simulator_fcv1table ${DIGITALCURLING_PLUGIN_TYPE}
    $<TARGET_OBJECTS:digitalcurling_simulator_fcv1table_obj>
    "./fcv1table.cpp"
)
target_include_directories(digitalcurling_simulator_fcv1table
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(digitalcurling_simulator_fcv1table
    PUBLIC  digitalcurling::plugin_api
    PRIVATE box2d
)
