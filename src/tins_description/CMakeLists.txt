cmake_minimum_required(VERSION 3.8)
project(tins_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
ament_package()
