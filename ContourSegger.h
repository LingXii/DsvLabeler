#include <vector>

#define MAXVALIDDIST	1.0


typedef struct  
{
	int x,y;
	double opti;
}PTOpti;

typedef struct
{
	int x,y;
}IMCOORDINATE;


int FindFirstValidPt(int scanno);
void calcRectOpti();
void ContourExtraction();
unsigned int RegionGrow(bool withOpti);
void GrowOne(IMCOORDINATE seed, unsigned int regionID, int & xMin, int & xMax);
void Region2Seg ();
