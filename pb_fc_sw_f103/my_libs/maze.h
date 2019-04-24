/******************************************************************************
File        : maze.h
Author      : Daniel Tar
Version     :
Copyright   :
Description : maze storing based on Andras Varga's idea
Info        : 14-09-2018
******************************************************************************/

#ifndef MAZE_H_
#define MAZE_H_

/*==========================================================================*/
/*                               Includes								 	*/
/*==========================================================================*/
#include <math.h>
#include "conf.h"
#include "util.h"
#include "eeprom.h"


/*==========================================================================*/
/*                            Private typedef								*/
/*==========================================================================*/


/*==========================================================================*/
/*                            Private define								*/
/*==========================================================================*/
#define DIR_SIZE 					2
#define MAZE_SIZE 					32
#define MAZE_VIRT_ADD				0x0000
#define MAZE_DATA_SIZE_IN_UINT16 	(2 * DIR_SIZE * MAZE_SIZE)

/*==========================================================================*/
/*                            Private macro									*/
/*==========================================================================*/

/*==========================================================================*/
/*                            Private variables								*/
/*==========================================================================*/

/*==========================================================================*/
/*                      Private function prototypes							*/
/*==========================================================================*/
void Init_EEPROM(void);
void Init_Maze(void);
uint16_t saveMaze(void);
uint16_t loadMaze(void);


void addWall(uint8_t x, uint8_t y, int8_t wall_type);
int8_t getWall(uint8_t x, uint8_t y, int8_t wall_type);



/*
uint32_t MazeWalls[33][2];
uint32_t Visited[32];

void AddVisited(uint8_t x, uint8_t y);
uint8_t GetVisited(uint8_t x, uint8_t y);
struct Position
{
short X;
short Y;
short Orientation;
} Position;
struct List open, shortestpath;
//struct NodeList nodes;
struct Node
{
short X;
short Y;
short distance;
struct Node *prev;
};
struct Node nodes[32][32];
struct Node nodes1[32][32];
struct List {
unsigned int NumOfElements;
struct ListElement * First;
struct ListElement * Last;
};
struct ListElement {
struct Node *Node;
struct ListElement * Next;
};
void RefreshPosition(short move);
void AddToList(struct List *List,struct Node *node);
char DeleteFromList(struct List *List);
void RelocateInList(struct List *List,struct Node *node);
void AddToListOrdered(struct List *List, struct Node *node);

short Next();
short CalcNextMove(struct Node *tempnode);
short CalcNextMove2(struct Node *tempnode);
void EvaluateNeighbour(struct Node *neighbour,struct Node *act);
char IsInList(struct List *List,struct Node *node);
short CalcDistance(struct Node *neighbour,struct Node *act);
short Plan(short startX, short startY, short goalX, short goalY);
char nextmove[256];
short movecounter;
char IsWithinBoudaries(short x,short y);
*/


#endif /* MAZE_H_ */
