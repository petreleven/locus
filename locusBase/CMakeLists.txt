add_library(LocusMathFunctions LocusMathFunctions.cpp)
add_library(Core Core.cpp)
target_link_libraries(LocusMathFunctions PUBLIC Core
)
