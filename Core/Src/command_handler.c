
#include <stm32g4xx_hal.h>
#include "command_handler.h"
#include "string.h"
#include "stdio.h"
#include "SERVO.h"
#include "stdlib.h"

int (*Commands[NUM_COMMANDS]) ( char** arguments);

struct parsed_command CommandHandler_ParseCommand(char* str){
    struct parsed_command parsedCommand;
    char* command;
    char* argument;
    int commandID;

    printf(" *** Begin Parsing Command *** \n\r*\n\r*\n\r*");
    command = strtok(str," -");

    // Handle Error
    if(command == NULL){
        parsedCommand.commandID = -1;
        return  parsedCommand;
    }

    printf("* Command: %s \n\r*\n\r*",command);
    commandID = CommandHandler_GetCommandID(command);

    // Handle Error
    if(commandID == -1){
        parsedCommand.commandID = -1;
        return  parsedCommand;
    }

    parsedCommand.commandID = CommandHandler_GetCommandID(command);

    for(int i = 0; i < 5; i++){
      argument = strtok(NULL," -");
      if (argument != NULL){
          parsedCommand.arguments[i] = argument;
          printf("* Argument %d: %s\n\r*\n\r*",i,argument);
      }
      else{
          printf("* No more arguments to parse \n\r*\n\r*");
          break;
      }
    }
    printf(" *** Finish Parsing Command ***\n\r\n\r ");
    return parsedCommand;
}

int CommandHandler_HandleCommand(struct parsed_command parsedCommand){
    Commands[parsedCommand.commandID](parsedCommand.arguments);
}

int CommandHandler_GetCommandID(char* command) {

    if (strcmp(command, "sweepfast") == 0 || strcmp(command, "sweepf") == 0 || strcmp(command, "sweep") == 0) {
        return 0;
    } else if (strcmp(command, "sweepslow") == 0 || strcmp(command, "sweeps") == 0) {
        return 1;
    } else if (strcmp(command, "clench") == 0) {
        return 2;
    } else if (strcmp(command, "release") == 0) {
        return 3;
    } else {
        return -1;
    }
}

int CommandHandler_Initialize(){

    Commands[COMMAND_SWEEP_SERVOS_FAST] = CommandHandler_ServoSweepFast;
    Commands[COMMAND_SWEEP_SERVOS_SLOW] = CommandHandler_ServoSweepSlow;
    Commands[COMMAND_CLENCH_SERVOS] = CommandHandler_ServoClench;
    Commands[COMMAND_RELEASE_SERVOS] = CommandHandler_ServoRelease;
    return 0;
}

int CommandHandler_ServoSweepFast(char** arguments){

    int servoID = atoi(arguments[0]);
    for (int i=100;i>10; i--) {

        SERVO_RawMove(servoID,i);
        HAL_Delay(6);
    }

    for (int i=10;i<100; i++){

        SERVO_RawMove(servoID,i);
        HAL_Delay(6);
    }
}

int CommandHandler_ServoSweepSlow(char** arguments){

    int servoID = atoi(arguments[0]);
    for (int i=100;i>10; i--) {

        SERVO_RawMove(servoID,i);
        HAL_Delay(15);
    }

    for (int i=10;i<100; i++){

        SERVO_RawMove(servoID,i);
        HAL_Delay(15);
    }
}

int CommandHandler_ServoClench(char** arguments){

    int servoID = atoi(arguments[0]);
    for (int i=100;i>10; i--) {

        SERVO_RawMove(servoID,i);
        HAL_Delay(10);
    }
}

int CommandHandler_ServoRelease(char** arguments){

    int servoID = atoi(arguments[0]);
    for (int i=10;i<100; i++){

        SERVO_RawMove(servoID,i);
        HAL_Delay(10);
    }
}