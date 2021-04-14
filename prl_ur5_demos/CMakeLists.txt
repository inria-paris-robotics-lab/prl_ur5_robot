cmake_minimum_required(VERSION 2.8.3)
project(prl_ur5_demos)

find_package(catkin REQUIRED)

catkin_package()

# install
foreach(dir models)
    install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

catkin_install_python(
    PROGRAMS scripts/group_info.py
             scripts/move_arm.py
             scripts/move_both_arms.py
             scripts/move_gripper.py
             scripts/pick_and_place.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
