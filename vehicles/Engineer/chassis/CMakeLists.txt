
project(Engineer ASM C CXX)

irm_add_arm_executable(Engineer_chassis
    TARGET DJI_Board_TypeC 
    SOURCES
    src/main.cc
    src/chassis_task.cc
    src/arm_translate_task.cc
    src/engineer_steering.cc
    src/steering_6020.cc
    src/selfcheck_task.cc
#     src/ui_task.cc
    INCLUDES include


)