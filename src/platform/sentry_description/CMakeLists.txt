cmake_minimum_required(VERSION 3.16)
project(sentry_description)

find_package(ament_cmake REQUIRED)

# Plain resource package: URDF + meshes, no build steps.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
