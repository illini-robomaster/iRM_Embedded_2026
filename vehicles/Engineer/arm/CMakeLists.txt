
project(Engineer ASM C CXX)

# If using DBUS, don't run arm and chassis at the same time
# comment one of the definitions in main.cc

# add_compile_definitions(USING_DBUS)

irm_add_arm_executable(Engineer_arm
        TARGET DM_MC_01
        SOURCES
        src/main.cc
        src/arm.cc
        src/selfcheck_task.cc
        src/ui_task.cc
        src/referee_task.cc
        INCLUDES include
)