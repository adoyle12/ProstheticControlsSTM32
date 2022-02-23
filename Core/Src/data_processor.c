#include "data_processor.h"
#include "sll.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include <command_handler.h>
#include <math.h>
#include <time.h>
#include <hand.h>
#include <main.h>

//List *lp;
//int DataProcessor_Initialize(){
//    printf("Initializing Linked List... \r\n");
//
//    lp = (List *) malloc(sizeof(List));
//    initList(lp);
//
//    printf("Initialized Linked List \r\n");
//
//}

//CSV
//int DataProcessor_ReadData(){
//    printf("Reading data..\r\n");
//    char* pch = NULL;
//
//    pch = strtok(mock_csv, "\n");
//
//    while (pch != NULL)
//    {
//        addAtTail(lp, atof(pch));
//        pch = strtok(NULL, "\n");
//    }
//    printf("Done reading data..\r\n");
//
//    return 0;
//}

//int DataProcessor_ReadData(uint16_t half_buffer[4096], int startIndex, int stopIndex){
//    printf("Reading data...\r\n");
//
//    while(delFromHead(lp) != -1){
//    }
//
//    int i;
//    for (i=startIndex; i<=stopIndex; i++)
//    {
//        addAtTail(lp, half_buffer[i]);
//    }
//    printf("Done reading data..\r\n");
//
//    DataProcessor_ProcessData();
//
//    return 0;
//}

//TODO: Add comments, not processing all 40 values
//int DataProcessor_ProcessData() {
//    printf("Processing data...\r\n");
//    int listLength = getListLength(lp);
//    int nodeIndex = 0;
//    float currentSum = 0;
//
//    //Calculate absolute value
//    Node *n = lp->head;
//    do {
//        Node *sumNode = n;
//
//        for (int i = 0; i < 40; i++) {
//            currentSum += fabs(sumNode->item);
//            sumNode = sumNode->next;
//            if (sumNode == NULL)
//                break;
//        }
//        n->item = currentSum / 40;
//        nodeIndex++;
//        n = n->next;
//        currentSum = 0;
//    } while (nodeIndex < listLength - 40);
//
//    //Remove last 40 nodes
//    for(int i=0; i<40; i++){
//        delFromTail(lp);
//    }
//    printf("Done processing data...\r\n");
//
//    DataProcessor_CheckThreshold();
//}

// TODO: ADD TIME DELAY
int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex){
//    printf("** Checking threshold..\r\n");
    int allFingers[] = {0, 1, 2, 3, 4};

    while (startIndex < stopIndex){
        if(half_buffer[startIndex] >= UPPER_THRESHOLD && FingerPositions[0] > MIN_FINGER_POSITION){
            Hand_Move(FingerPositions[0] + 1, allFingers);
            printf("%i passed upper threshold of %i. Clenching... \r\n", half_buffer[startIndex], UPPER_THRESHOLD);
        } else if(half_buffer[startIndex] < LOWER_THRESHOLD && FingerPositions[0] > MIN_FINGER_POSITION){
            Hand_Move(FingerPositions[0] - 1, allFingers);
            printf("%i passed lower threshold of %i. Releasing... \r\n", half_buffer[startIndex], LOWER_THRESHOLD);
        }
        startIndex++;
    }
}