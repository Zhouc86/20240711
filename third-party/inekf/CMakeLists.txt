
find_package (Eigen3 REQUIRED)

add_library(inekf SHARED InEKF.h InEKF.cpp LieGroup.h LieGroup.cpp NoiseParams.h NoiseParams.cpp RobotState.h RobotState.cpp)
target_link_libraries (inekf Eigen3::Eigen)
