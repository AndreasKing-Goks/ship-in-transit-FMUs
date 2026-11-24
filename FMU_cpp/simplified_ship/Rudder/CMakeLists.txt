include(initModel)

set(SYSTEM_HEADER_NAME "rudder.h")
set(SYS_H ${CMAKE_CURRENT_SOURCE_DIR}/include/${SYSTEM_HEADER_NAME})
set(SYS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/rudder.cpp)
set(RESRC "")
set(UNITTEST_SRC "")

include(modelTargets)
