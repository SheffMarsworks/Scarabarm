cmake_minimum_required(VERSION 3.8)
project(scarabarm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY meshes urdf launch worlds
  DESTINATION share/${PROJECT_NAME}
)



# Automates the process of setting up linting for the package, which
# is the process of running tools that analyze the code for potential
# errors, style issues, and other discrepancies that do not adhere to
# specified coding standards or best practices.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
 
ament_package()