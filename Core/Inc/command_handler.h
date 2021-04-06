#ifndef PROSTHETICCONTROLSSTM32_COMMAND_HANDLER_H
#define PROSTHETICCONTROLSSTM32_COMMAND_HANDLER_H



#define COMMAND_SWEEP_SERVOS 0
#define NUM_COMMANDS 1


typedef struct parsed_command {
    int commandID;
    char* arguments[5];
};

int (*Commands[NUM_COMMANDS]) ( char** arguments);
struct parsed_command CommandHandler_ParseCommand(char* str);
int CommandHandler_HandleCommand(struct parsed_command parsedCommand);
int CommandHandler_GetCommandID(char* command);
int CommandHandler_Initialize();
int CommandHandler_ServoSweep(char** arguments);

#endif PROSTHETICCONTROLSSTM32_COMMAND_HANDLER_H