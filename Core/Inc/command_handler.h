#ifndef PROSTHETICCONTROLSSTM32_COMMAND_HANDLER_H
#define PROSTHETICCONTROLSSTM32_COMMAND_HANDLER_H



#define COMMAND_SWEEP_SERVOS_FAST 0
#define COMMAND_SWEEP_SERVOS_SLOW 1
#define NUM_COMMANDS 2


typedef struct parsed_command {
    int commandID;
    char* arguments[5];
};


struct parsed_command CommandHandler_ParseCommand(char* str);
int CommandHandler_HandleCommand(struct parsed_command parsedCommand);
int CommandHandler_GetCommandID(char* command);
int CommandHandler_Initialize();
int CommandHandler_ServoSweepFast(char** arguments);
int CommandHandler_ServoSweepSlow(char** arguments);

#endif PROSTHETICCONTROLSSTM32_COMMAND_HANDLER_H