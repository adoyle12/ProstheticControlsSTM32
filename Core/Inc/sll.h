#ifndef PROSTHETICCONTROLSSTM32_SLL_H
#define PROSTHETICCONTROLSSTM32_SLL_H

#include <stdio.h>
#include <stdlib.h>

//Self referential structure to create node.
typedef struct tmp
{
    float item;
    struct tmp *next;
}Node;


//structure for create linked list.
typedef struct
{
    Node * head;
    Node * tail;
}List;

//Initialize List
void initList(List * lp);

//Create node and return reference of it.
Node * createNode(float item);

//Add new item at the end of list.
void addAtTail(List * lp,float item);

//Delete item from the end of list.
float delFromTail(List * lp);

//Add new item at begning of the list.
void addAtHead(List * lp,float item);

//Delete item from Start of list.
float delFromHead(List * lp);

//To print list from start to end of the list.
void printList(List *lp);

//Get list count
int getListLength(List *lp);

#endif PROSTHETICCONTROLSSTM32_SLL_H
