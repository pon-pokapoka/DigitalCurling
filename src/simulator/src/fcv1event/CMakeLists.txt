
# --- Box2D ---
set(DCSIM_FCV1Event_BOX2D_VERSION "v2.4.1")
include(FetchContent)
FetchContent_Declare(
    box2d
    GIT_REPOSITORY "https://github.com/erincatto/box2d.git"
    GIT_TAG        ${DCSIM_FCV1Event_BOX2D_VERSION}
    PATCH_COMMAND  ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/patch_box2d.cmake"
    SYSTEM
)
set(BOX2D_BUILD_TESTBED OFF)
set(BOX2D_BUILD_UNIT_TESTS OFF)
FetchContent_MakeAvailable(box2d)

# --- Build plugin object ---
add_library(digitalcurling_simulator_fcv1event_obj OBJECT
    "./simulator_fcv1event.cpp"
    "./simulator_fcv1event_factory.cpp"
    "./simulator_fcv1event_storage.cpp"
    "./box2d_frames.cpp"
    "./contact_frame.cpp"
    "./event_search.cpp"
    "./free_motion_curve.cpp"
    "./free_path.cpp"
)
target_include_directories(digitalcurling_simulator_fcv1event_obj
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(digitalcurling_simulator_fcv1event_obj
    PUBLIC  digitalcurling::plugin_api
    PUBLIC  box2d  # the tests include the header of fcv1, which needs it
)

# --- Build plugin ---
add_library(digitalcurling_simulator_fcv1event ${DIGITALCURLING_PLUGIN_TYPE}
    $<TARGET_OBJECTS:digitalcurling_simulator_fcv1event_obj>
    "./fcv1event.cpp"
)
target_include_directories(digitalcurling_simulator_fcv1event
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(digitalcurling_simulator_fcv1event
    PUBLIC  digitalcurling::plugin_api
    PRIVATE box2d
)
