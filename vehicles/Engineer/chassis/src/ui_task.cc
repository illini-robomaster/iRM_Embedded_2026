#include "ui_task.h"


void UITask(void* arg){
    UNUSED(arg);

    UI->SetID(referee->game_robot_status.robot_id);


    communication::package_t frame;
    communication::graphic_data_t graphCrosshair1;
    communication::graphic_data_t graphCrosshair2;
    communication::graphic_data_t graphCrosshair3;
    communication::graphic_data_t graphCrosshair4;
    communication::graphic_data_t graphCrosshair5;
    communication::graphic_data_t graphCrosshair6;
    communication::graphic_data_t graphCrosshair7;

    // char msgBuffer1[30] = "TEST";

    // Initialize crosshair GUI

    while (true) {
        UI->CrosshairGUI(&graphCrosshair1, &graphCrosshair2, &graphCrosshair3, &graphCrosshair4,
                    &graphCrosshair5, &graphCrosshair6, &graphCrosshair7);
        UI->GraphRefresh((uint8_t*)(&referee->graphic_seven), 7, graphCrosshair1, graphCrosshair2,
                    graphCrosshair3, graphCrosshair4, graphCrosshair5, graphCrosshair6,
                    graphCrosshair7);
        referee->PrepareUIContent(communication::SEVEN_GRAPH);
        frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
        referee_uart->Write(frame.data, frame.length);
        osDelay(UI_TASK_DELAY);

    }


}


void init_ui(){
    UI = new communication::UserInterface();



}

void kill_ui(){

}