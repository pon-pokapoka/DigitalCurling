set(TARGET_FILE "CMakeLists.txt")

file(READ ${TARGET_FILE} ROOT_CONTENT)
string(REGEX REPLACE "cmake_minimum_required[ \t]*\\(VERSION[ \t]+3\\.[0-9]+\\)"
                     "cmake_minimum_required(VERSION 3.15)"
                      ROOT_MODIFIED "${ROOT_CONTENT}")

if(ROOT_MODIFIED STREQUAL ROOT_CONTENT)
    message(WARNING "Box2D Root CMakeLists.txt: Patch pattern not found!")
else()
    file(WRITE ${TARGET_FILE} "${ROOT_MODIFIED}")
endif()

# Box2D times every phase of every step with b2Timer, only to fill b2Profile,
# which nothing here reads. Those clock calls took about half of fcv1light's
# time. Select b2_timer.cpp's own no-op branch; results don't change.
set(TIMER_FILE "src/common/b2_timer.cpp")

file(READ ${TIMER_FILE} TIMER_CONTENT)
string(REPLACE "#if defined(_WIN32)" "#if 0 // patched: timers disabled"
               TIMER_MODIFIED "${TIMER_CONTENT}")
string(REPLACE "#elif defined(__linux__) || defined (__APPLE__)" "#elif 0 // patched: timers disabled"
               TIMER_MODIFIED "${TIMER_MODIFIED}")

if(NOT TIMER_MODIFIED STREQUAL TIMER_CONTENT)
    file(WRITE ${TIMER_FILE} "${TIMER_MODIFIED}")
elseif(NOT TIMER_CONTENT MATCHES "patched: timers disabled")
    message(WARNING "Box2D b2_timer.cpp: Patch pattern not found!")
endif()
