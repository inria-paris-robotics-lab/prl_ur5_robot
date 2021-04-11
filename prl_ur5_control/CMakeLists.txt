cmake_minimum_required(VERSION 2.8.3)
project(prl_ur5_control)

find_package(catkin REQUIRED)

catkin_package()

# install
foreach(dir config launch)
    install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
