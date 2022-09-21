cmake_minimum_required(VERSION 3.5)

project(turtle_draw)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

file(GLOB scripts src/*)
install(PROGRAMS ${scripts} DESTINATION lib/${PROJECT_NAME})

install(FILES "HRP-2.jpg" DESTINATION share/${PROJECT_NAME})

ament_package()
