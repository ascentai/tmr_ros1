cmake_minimum_required(VERSION 2.8.3)
project(tm_calibration)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS script/calibration_correction.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
