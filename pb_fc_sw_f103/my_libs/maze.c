/******************************************************************************
File        : maze.c
Author      : Andras Varga, modified by Daniel Tar
Version     :
Copyright   :
Description :
Info        : 14-09-2018
******************************************************************************/

/*==========================================================================*/
/*                               Includes								 	*/
/*==========================================================================*/
#include "maze.h"

/*==========================================================================*/
/*                            Private typedef								*/
/*==========================================================================*/


/*==========================================================================*/
/*                            Private define								*/
/*==========================================================================*/


/*==========================================================================*/
/*                            Private macro									*/
/*==========================================================================*/


/*==========================================================================*/
/*                            Private variables								*/
/*==========================================================================*/

uint16_t VarValue2 = 0;
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab2[3] = {0x5555, 0x6666, 0x7777};

/* mazeWalls arrays description
 *
 *  the mazeWalls array stores the information from the walls in a very complex way, to
 *  minimalize the flash memory costs
 *
 *  there are two types of walls, horizontal and vertical
 *  the second index shows which kind of wall it is:
 *  	0 - horizontal
 *  	1 - vertical
 *
 *  both from vertical and horizontal walls are 32 pieces and if there is a wall
 *  then the uint32's bit is set to 1
 *
 */
uint32_t mazeWalls[DIR_SIZE][MAZE_SIZE];

/*==========================================================================*/
/*                      Private function prototypes							*/
/*==========================================================================*/


/*==========================================================================*/
/*                          Private functions								*/
/*==========================================================================*/

void Init_EEPROM(){
	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();
	/* EEPROM Init */
	EE_Init();
	/* Lock the Flash Program Erase controller */
	FLASH_Lock();
}

void Init_Maze(){
	mazeWalls[0][0] = 0xffffffff;
	mazeWalls[1][0] = 0xffffffff;
	mazeWalls[0][MAZE_SIZE-1] = 0xffffffff;
	mazeWalls[1][MAZE_SIZE-1] = 0xffffffff;
}

/* returns 0 if all data are written successfully */
uint16_t saveMaze(){
	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();

	uint16_t tempValue = 0x0000;

	for(uint16_t dir = 0; dir < DIR_SIZE; ++dir){
		for(uint16_t cell; cell < MAZE_SIZE; ++cell){
			/* write MSB half of the uint32_t variable to the eeprom */
			tempValue = (( mazeWalls[dir][cell] & 0xFFFF0000) >> 16);
			if(EE_WriteVariable( dir*cell*2 , tempValue) != FLASH_COMPLETE){
				return 1;
			}

			/* write LSB half of the uint32_t variable to the eeprom */
			tempValue = ( mazeWalls[dir][cell] & 0x0000FFFF);
			if(EE_WriteVariable( dir*cell*2 + 1, tempValue)!= FLASH_COMPLETE){
				return 1;
			}
		}
	}

	FLASH_Lock();
	return 0; /* returns 0 if all data are written successfully */
}

/* returns 0 if all data read successfully */
uint16_t loadMaze(){

	uint16_t readValue = 0x0000;
	uint16_t failCounter = 0;

	for(uint16_t dir = 0; dir < DIR_SIZE; ++dir){
		for(uint16_t cell; cell < MAZE_SIZE; ++cell){

			readValue = 0x0000;

			/* read MSB half of the uint32_t variable to the eeprom */
			failCounter += EE_ReadVariable( dir*cell*2 , &readValue);
			mazeWalls[dir][cell] |= (uint32_t)readValue << 16;

			readValue = 0x0000;

			/* read LSB half of the uint32_t variable to the eeprom */
			failCounter += EE_ReadVariable( dir*cell*2 + 1 , &readValue);
			mazeWalls[dir][cell] |= (uint32_t)readValue;
		}
	}

	return failCounter; /* returns 0 if all data read successfully */
}



void addWall(uint8_t x, uint8_t y, int8_t wall_type){
	switch (wall_type) {
		case 0:
			mazeWalls[1][x] |= (1 << y);
			break;
		case 1:
			mazeWalls[0][y] |= (1 << x);
			break;
		case 2:
			mazeWalls[1][x+1] |= (1 << y);
			break;
		case 3:
			mazeWalls[0][y+1] |= (1 << x);
			break;
	}
}

int8_t getWall(uint8_t x, uint8_t y, int8_t wall_type){
	switch (wall_type) {
		case 0:
			return (( mazeWalls[1][x] >> y ) & 0x01 );
		case 1:
			return (( mazeWalls[0][y] >> x ) & 0x01 );
		case 2:
			return (( mazeWalls[1][x+1] >> y ) & 0x01 );
		case 3:
			return (( mazeWalls[0][y+1] >> x ) & 0x01 );
	}
	return -1;
}

/*
void AddVisited(unsigned short x, unsigned short y)
{
Visited[x] |= 1 << y;
}
unsigned char GetVisited(unsigned short x, unsigned short y)
{
return ( Visited[x] >> y ) & 0x01;
}
void RefreshPosition(short retval)
{
switch (retval) {
case 2:
Position.Orientation--;
break;
case 3:
Position.Orientation++;
break;
case 4:
Position.Orientation+=2;
break;
}
if(Position.Orientation>3)
Position.Orientation-=4;
else if(Position.Orientation<0)
Position.Orientation+=4;
if(retval!=0)
switch (Position.Orientation) {
case 0:
Position.X--;
break;
case 2:
Position.X++;
break;
case 1:
Position.Y--;
break;
case 3:
Position.Y++;
break;
}
AddVisited(Position.X,Position.Y);
}
short Next()
{
for (int i=0;i<32;i++)
for(int j=0;j<32;j++)
{
nodes[i][j].X=i;
nodes[i][j].Y=j;
nodes[i][j].distance=0;
nodes[i][j].prev=NULL;
}
if(shortestpath.NumOfElements==0)
return 0;
else if(shortestpath.NumOfElements==1 && shortestpath.First->Node->X==Position.X &&
shortestpath.First->Node->Y==Position.Y)
return 0;
struct Node *n1=&nodes[Position.X][Position.Y];
AddToListOrdered(&open,n1);
while(open.NumOfElements != 0)
{
struct Node *actnode=open.First->Node;
if(IsInList(&shortestpath,actnode))
{
struct Node *tempnode=actnode;
while(!(tempnode->prev->X==Position.X && tempnode->prev->Y==Position.Y))
tempnode=tempnode->prev;
short ret=CalcNextMove(tempnode);
while(DeleteFromList(&open));
return ret;
}
DeleteFromList(&open);
if(IsWithinBoudaries(actnode->X-1,actnode->Y) && !GetWall(actnode->X,actnode->Y,0))
{
struct Node *n=&nodes[actnode->X-1][actnode->Y];
EvaluateNeighbour(n,actnode);
}
if(IsWithinBoudaries(actnode->X+1,actnode->Y) && !GetWall(actnode->X,actnode->Y,2))
{
struct Node *n=&nodes[actnode->X+1][actnode->Y];
EvaluateNeighbour(n,actnode);
}
if(IsWithinBoudaries(actnode->X,actnode->Y-1) && !GetWall(actnode->X,actnode->Y,1))
{
struct Node *n=&nodes[actnode->X][actnode->Y-1];
EvaluateNeighbour(n,actnode);
}
if(IsWithinBoudaries(actnode->X,actnode->Y+1) && !GetWall(actnode->X,actnode->Y,3))
{
struct Node *n=&nodes[actnode->X][actnode->Y+1];
EvaluateNeighbour(n,actnode);
}
actnode->distance=-1;
}
while(DeleteFromList(&open));
return 8;
}
short CalcNextMove(struct Node *tempnode)
{
switch (Position.Orientation) { //0-xminusz ,1-yminusz,2-xplusz,3-yplusz
case 0:
if(tempnode->X<tempnode->prev->X)
return 1;
if(tempnode->X>tempnode->prev->X)
return 4;
if(tempnode->Y>tempnode->prev->Y)
return 2;
if(tempnode->Y<tempnode->prev->Y)
return 3;
break;
case 2:
if(tempnode->X<tempnode->prev->X)
return 4;
if(tempnode->X>tempnode->prev->X)
return 1;
if(tempnode->Y>tempnode->prev->Y)
return 3;
if(tempnode->Y<tempnode->prev->Y)
return 2;
break;
case 1:
if(tempnode->X<tempnode->prev->X)
return 2;
if(tempnode->X>tempnode->prev->X)
return 3;
if(tempnode->Y>tempnode->prev->Y)
return 4;
if(tempnode->Y<tempnode->prev->Y)
return 1;
break;
case 3:
if(tempnode->X<tempnode->prev->X)
return 3;
if(tempnode->X>tempnode->prev->X)
return 2;
if(tempnode->Y>tempnode->prev->Y)
return 1;
if(tempnode->Y<tempnode->prev->Y)
return 4;
break;
}
return -1;
}
short CalcNextMove2(struct Node *tempnode)
{
short x=(tempnode->prev->X -tempnode->X)+(tempnode->prev->X-tempnode->prev->prev->X);
short y=(tempnode->prev->Y -tempnode->Y)+(tempnode->prev->Y-tempnode->prev->prev->Y);
if(x==0 && y==0)
return 1;
else if(abs(x)==2 || abs(y)==2)
return 4;
if(tempnode->prev->X>tempnode->prev->prev->X)
{
if(tempnode->Y>tempnode->prev->Y)
return 3;
else return 2;
}
if(tempnode->prev->X<tempnode->prev->prev->X)
{
if(tempnode->Y>tempnode->prev->Y)
return 2;
else return 3;
}
if(tempnode->prev->Y>tempnode->prev->prev->Y)
{
if(tempnode->X>tempnode->prev->X)
return 2;
else return 3;
}
if(tempnode->prev->Y<tempnode->prev->prev->Y)
{
if(tempnode->X>tempnode->prev->X)
return 3;
else return 2;
}
return -1;
}
void EvaluateNeighbour(struct Node *neighbour,struct Node *act)
{
if(neighbour->distance!=-1)
{
short dist=act->distance+CalcDistance(neighbour,act);
if(!IsInList(&open,neighbour))
{
neighbour->distance=dist;
neighbour->prev=act;
AddToListOrdered(&open,neighbour);
}
else {
if(dist<neighbour->distance)
{
neighbour->distance=dist;
neighbour->prev=act;
RelocateInList(&open,neighbour);
}
}
}
}
void EvaluateNeighbour2(struct Node *neighbour,struct Node *act)
{
if(neighbour->distance!=-1)
{
short dist=act->distance+CalcDistance(neighbour,act);
if(!IsInList(&open,neighbour))
{
neighbour->distance=dist;
neighbour->prev=act;
AddToListOrdered(&open,neighbour);
}
else {
if(dist<neighbour->distance)
{
neighbour->distance=dist;
neighbour->prev=act;
RelocateInList(&open,neighbour);
}
}
}
}
short CalcDistance(struct Node *neighbour,struct Node *act)
{
if(act->prev==NULL) //0-xminusz ,1-yminusz,2-xplusz,3-yplusz
switch (Position.Orientation) {
case 1:
if(act->X==neighbour->X && act->Y>neighbour->Y)
return 1;
else return 2;
break;
case 3:
if(act->X==neighbour->X && act->Y<neighbour->Y)
return 1;
else return 2;
break;
case 0:
if(act->X>neighbour->X && act->Y==neighbour->Y)
return 1;
else return 2;
break;
case 2:
if(act->X<neighbour->X && act->Y==neighbour->Y)
return 1;
else return 2;
break;
default:
break;
}
else
{
short x=(neighbour->X-act->X)+(act->prev->X-act->X);
short y=(neighbour->Y-act->Y)+(act->prev->Y-act->Y);
if(x==0 && y==0)
return 1;
else
return 2;
}
return -1;
}
short CalcDistance2(struct Node *neighbour,struct Node *act)
{
if(act->prev==NULL) //0-xminusz ,1-yminusz,2-xplusz,3-yplusz
switch (2) {
case 1:
if(act->X==neighbour->X && act->Y>neighbour->Y)
return 1;
else return 2;
break;
case 3:
if(act->X==neighbour->X && act->Y<neighbour->Y)
return 1;
else return 2;
break;
case 0:
if(act->X>neighbour->X && act->Y==neighbour->Y)
return 1;
else return 2;
break;
case 2:
if(act->X<neighbour->X && act->Y==neighbour->Y)
return 1;
else return 2;
break;
default:
break;
}
else
{
short x=(neighbour->X-act->X)+(act->prev->X-act->X);
short y=(neighbour->Y-act->Y)+(act->prev->Y-act->Y);
if(x==0 && y==0)
return 1;
else
return 2;
}
return -1;
}
short Plan(short startX, short startY,short goalX,short goalY)
{
while(DeleteFromList(&shortestpath));
for(int i=0;i<256;i++)
nextmove[i]=0;
for (int i=0;i<32;i++)
for(int j=0;j<32;j++)
{
nodes1[i][j].X=i;
nodes1[i][j].Y=j;
nodes1[i][j].distance=0;
nodes1[i][j].prev=NULL;
}
if(startX==goalX && startY==goalY)
{
return 0;
}
struct Node *n1=&nodes1[startX][startY];
AddToListOrdered(&open,n1);
while(open.NumOfElements != 0)
{
struct Node *actnode=open.First->Node;
if(actnode->X==goalX && actnode->Y==goalY)
{
struct Node *tempnode=actnode;
movecounter=0;
while(!(tempnode->prev->X==startX && tempnode->prev->Y==startY))
{
nextmove[movecounter]=CalcNextMove2(tempnode);
movecounter++;
if(!GetVisited(tempnode->X,tempnode->Y))
AddToList(&shortestpath,tempnode);
tempnode=tempnode->prev;
}
if(!GetVisited(tempnode->X,tempnode->Y))
AddToList(&shortestpath,tempnode);
nextmove[movecounter]=CalcNextMove(tempnode);
movecounter++;
for(int i=0;i<(movecounter+1)/2;i++)
{
char temp=nextmove[i];
nextmove[i]=nextmove[movecounter-1-i];
nextmove[movecounter-1-i]=temp;
}
while(DeleteFromList(&open));
char temp[250]={0};
for(int i=0;i<250;i++)
{
temp[i]=nextmove[i];
if(nextmove[i]==0 && nextmove[i+1]==0)
break;
}
int tempcounter=0;;
for(int i=0;i<250;i++)
{
nextmove[tempcounter]=temp[i];
if(temp[i]>1)
{
tempcounter++;
nextmove[tempcounter]=1;
movecounter++;
}
tempcounter++;
if(temp[i]==0 && temp[i+1]==0)
break;
temp[i]=0;
}
int tc1=1;
if(nextmove[0]==1) {temp[0]=1;temp[1]=1;tc1=2;}
else{temp[0]=nextmove[0],tc1=1;}
for(int i=1;i<movecounter;i++)
{
if(nextmove[i]!=1)
{
temp[tc1]=nextmove[i];
tc1++;
}
else if(nextmove[i-1]!=1)
{
temp[tc1]=1;
temp[tc1+1]=1;
tc1+=2;
}
else temp[tc1-1]++;
}
tc1=0;
while(temp>0 && tc1<256)
{nextmove[tc1]=temp[tc1];tc1++;}
return 1;
}
DeleteFromList(&open);
if(IsWithinBoudaries(actnode->X-1,actnode->Y) && !GetWall(actnode->X,actnode->Y,0))
{
struct Node *n=&nodes1[actnode->X-1][actnode->Y];
EvaluateNeighbour2(n,actnode);
}
if(IsWithinBoudaries(actnode->X+1,actnode->Y) && !GetWall(actnode->X,actnode->Y,2))
{
struct Node *n=&nodes1[actnode->X+1][actnode->Y];
EvaluateNeighbour2(n,actnode);
}
if(IsWithinBoudaries(actnode->X,actnode->Y-1) && !GetWall(actnode->X,actnode->Y,1))
{
struct Node *n=&nodes1[actnode->X][actnode->Y-1];
EvaluateNeighbour2(n,actnode);
}
if(IsWithinBoudaries(actnode->X,actnode->Y+1) && !GetWall(actnode->X,actnode->Y,3))
{
struct Node *n=&nodes1[actnode->X][actnode->Y+1];
EvaluateNeighbour2(n,actnode);
}
actnode->distance=-1;
}
while(DeleteFromList(&open));
return 0;
}
void AddToList(struct List *List, struct Node *node) {
struct ListElement *NewElement = malloc(sizeof(struct ListElement));
NewElement->Node = node;
NewElement->Next = NULL;
if (List->First == NULL) {
List->First = NewElement;
List->Last = NewElement;
}
else {
List->Last->Next = NewElement;
List->Last = NewElement;
}
List->NumOfElements++;
}
char IsInList(struct List *List,struct Node *node)
{
struct ListElement *current=List->First;
while(current!=NULL)
{
if(current->Node->X==node->X && current->Node->Y==node->Y)
return 1;
current=current->Next;
}
return 0;
}
void AddToListOrdered(struct List *List, struct Node *node) {
struct ListElement *NewElement = malloc(sizeof(struct ListElement));
NewElement->Node = node;
NewElement->Next = NULL;
if (List->First == NULL) {
List->First = NewElement;
List->Last = NewElement;
}
else if (node->distance<List->First->Node->distance) {
NewElement->Next=List->First;
List->First = NewElement;
}
else if (node->distance>=List->Last->Node->distance)
{
List->Last->Next = NewElement;
List->Last = NewElement;
}
else
{
struct ListElement *act=List->First->Next;
struct ListElement *prev=List->First;
short d=node->distance;
do {
if(d < act->Node->distance)
{
prev->Next=NewElement;
NewElement->Next=act;
break;
}
else {
prev=act;
act=act->Next;
}
} while (1);
}
List->NumOfElements++;
}
void RelocateInList(struct List *List,struct Node *node)
{
struct ListElement *act=List->First;
struct ListElement *element;
if(List->First->Node==node)
{
element=List->First;
List->First=List->First->Next;
}
else
while(act->Next!=NULL)
{
if(act->Next->Node==node)
{
element=act->Next;
act->Next=act->Next->Next;
if (element == List->Last) {
List->Last = act;
}
break;
}
act=act->Next;
}
element->Next = NULL;
if (List->First == NULL) {
List->First = element;
List->Last = element;
}
else if (node->distance<List->First->Node->distance) {
element->Next=List->First;
List->First = element;
}
else if (node->distance>=List->Last->Node->distance)
{
List->Last->Next = element;
List->Last = element;
}
else
{
struct ListElement *act=List->First->Next;
struct ListElement *prev=List->First;
short d=node->distance;
do {
if(d < act->Node->distance)
{
prev->Next=element;
element->Next=act;
break;
}
else {
prev=act;
act=act->Next;
}
} while (1);
}
}
char DeleteFromList(struct List *List) {
if (List->First != NULL) {
struct ListElement *Delete = List->First;
List->First = List->First->Next;
free(Delete);
List->NumOfElements--;
if (List->NumOfElements == 0) {
List->First = NULL;
List->Last = NULL;
return 0;
}
return 1;
}
else return 0;
}
char IsWithinBoudaries(signed short x,signed short y)
{
if (x<0 || x>31 || y<0 || y >31)
return 0;
else
return 1;
}*/

