#include "DsvLabelMv.h"
#include "ContourSegger.h"

#include <vector>
using namespace std;


vector <PTOpti> VecOpti;


#define ELEVATION	1.0
#define RADIUS		1
#define FITERROR	1
#define	COS75		0.258819

#define REGIONNUM		50000

int SegX[REGIONNUM][2];

bool AddPoints(IMCOORDINATE seed,vector<IMCOORDINATE> & vec, int _regid, int & x1, int & x2)
{
	if (rm.regionID[rm.wid*seed.x +seed.y] == UNKNOWN)
		rm.regionID[rm.wid*seed.x +seed.y] = _regid;
	else if (rm.regionID[rm.wid*seed.x +seed.y] != _regid)
		return false;

	x1 = x2 = seed.x;

	IMCOORDINATE FourNeighbor;

	FourNeighbor.x = seed.x - 1;
	FourNeighbor.y = seed.y;
	if (FourNeighbor.x>0&&FourNeighbor.x<rm.len&&
		FourNeighbor.y>0&&FourNeighbor.y<rm.wid&&
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] == UNKNOWN)
	{
		vec.push_back(FourNeighbor);
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] = _regid;
		x1 = seed.x - 1;
	}

	FourNeighbor.x = seed.x + 1;
	FourNeighbor.y = seed.y;
	if (FourNeighbor.x>0&&FourNeighbor.x<rm.len&&
		FourNeighbor.y>0&&FourNeighbor.y<rm.wid&&
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] == UNKNOWN)
	{
		vec.push_back(FourNeighbor);
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] = _regid;
	}	x2 = seed.x + 1;

	FourNeighbor.x = seed.x;
	FourNeighbor.y = seed.y - 1;
	if (FourNeighbor.x>0&&FourNeighbor.x<rm.len&&
		FourNeighbor.y>0&&FourNeighbor.y<rm.wid&&
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] == UNKNOWN)
	{
		vec.push_back(FourNeighbor);
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] = _regid;
	}

	FourNeighbor.x = seed.x;
	FourNeighbor.y = seed.y + 1;
	if (FourNeighbor.x>0&&FourNeighbor.x<rm.len&&
		FourNeighbor.y>0&&FourNeighbor.y<rm.wid&&
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] == UNKNOWN)
	{
		vec.push_back(FourNeighbor);
		rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] = _regid;
	}
	
	return true;
}

void ContourExtraction ()
{
	point3fi *cp,*pt;
	double dis,maxdis;
	int x,y,xx,yy;
	int validCount;

	for (x = 0; x<rm.len; x++)
	{
		for (y = 0; y<rm.wid; y++)
		{
			cp = &rm.pts[rm.wid*x+y];
			if (!cp->i)
			{
				rm.regionID[rm.wid*x +y] = NONVALID;
				continue;
			}
			else if (rm.segflg[rm.wid*x+y] == ROADFLG)
			{
				rm.regionID[rm.wid*x +y] = GROUND;
				continue;
			}
			else if (rm.segflg[rm.wid*x+y] == HIGHFLG)
			{
				rm.regionID[rm.wid*x +y] = BACKGROUND;
				continue;
			}
			validCount = 0;
			for (xx = x-1; xx<=x+1; xx++)
			{
				if (xx<0||xx>=rm.len)
					continue;
				for (yy = y-1; yy<=y+1; yy++)
				{
					if(yy<0||yy>=rm.wid)
						continue;
					if (x==xx&&y==yy)
						continue;
					if (x!=xx&&y!=yy)
						continue;

					pt = &rm.pts[rm.wid*xx+yy];
					if (!pt->i||rm.segflg[rm.wid*xx+yy]==ROADFLG||rm.segflg[rm.wid*xx+yy]==HIGHFLG)
						validCount++;
				}
			}
			if(validCount>1) {
				rm.regionID[rm.wid*x +y] = EDGEPT;
				continue;
			}

			yy = y;
			maxdis = 0;
			for (xx = x-1; xx<=x+1; xx++)
			{
				if (xx<0||xx>=rm.len||xx==x)
					continue;

				pt = &rm.pts[rm.wid*xx+yy];
				if (!pt->i||rm.segflg[rm.wid*xx+yy]==ROADFLG||rm.segflg[rm.wid*xx+yy]==HIGHFLG)
					continue;
				dis = ppDistance3fi(cp,pt);
				maxdis = max (maxdis,dis);
			}
			if (maxdis>=BOUND(VERTERRFACTOR*p2r(cp),BASEERROR,MAXDISTHRE))
				rm.regionID[rm.wid*x +y] = EDGEPT;

			xx = x;
			maxdis = 0;
			for (yy = y-1; yy<=y+1; yy++)
			{
				if(yy<0||yy>=rm.wid||yy==y)
					continue;

				pt = &rm.pts[rm.wid*xx+yy];
				if (!pt->i||rm.segflg[rm.wid*xx+yy]==ROADFLG||rm.segflg[rm.wid*xx+yy]==HIGHFLG)
					continue;
				dis = ppDistance3fi(cp,pt);
				maxdis = max (maxdis,dis);
			}
			if (maxdis>=BOUND(HORIERRFACTOR*p2r(cp),BASEERROR,MAXDISTHRE))
				rm.regionID[rm.wid*x +y] = EDGEPT;
		}
	}
}

void GrowOne(IMCOORDINATE seed, unsigned int _regid, int & xMin, int & xMax)
{
	vector <IMCOORDINATE> Vec1;
	vector <IMCOORDINATE> Vec2;
	bool isVec1 = true;
	int		j;
	int		x1, x2;
	IMCOORDINATE	FourNeighbor;
	Vec1.clear();
	Vec2.clear();
	Vec1.push_back(seed);
	xMin = xMax = seed.x;
	while (1)
	{
		switch (isVec1)
		{
		case true:
			Vec2.clear();
			for (j =0; j<Vec1.size(); j++)
			{
				FourNeighbor.x = Vec1[j].x;
				FourNeighbor.y = Vec1[j].y;
				AddPoints(Vec1[j], Vec2, _regid, x1, x2);
				if (x1<xMin)
					xMin = x1;
				if (x2>xMax)
					xMax = x2;
			}
			isVec1 = !isVec1;
			if (Vec2.size()<1)
			{
				return;
			}
			break;
		case false:
			Vec1.clear();
			for (j =0; j<Vec2.size(); j++)
			{
				FourNeighbor.x = Vec2[j].x;
				FourNeighbor.y = Vec2[j].y;
				AddPoints(Vec2[j], Vec1, _regid, x1, x2);
				if (x1<xMin)
					xMin = x1;
				if (x2>xMax)
					xMax = x2;
			}
			isVec1 = !isVec1;
			if (Vec1.size()<1)
			{
				return;
			}
			break;
		}
	}
}

unsigned int RegionGrow()
{
	IMCOORDINATE	seed;
	int m,x1,x2;
    unsigned int _regid = 1;

	int x,y,yf;
	for (x=0; x<rm.len; x++)
	{
		for (yf = 0; yf<rm.wid; yf++) {
			if (rm.pts[rm.wid*x+yf].i)
//			if (rm.pts[yf].i)			//this error was corrected on Mar.25,2018
				break;
		}
		if(yf >= rm.wid)
			continue;
		for (y = yf; y<rm.wid; y++)
		{
			if (rm.regionID[rm.wid*x+y]==UNKNOWN)
			{
				seed.x = x;
				seed.y = y;
				GrowOne(seed, _regid, x1, x2);
				SegX[_regid][0]=x1;
				SegX[_regid][1]=x2;
				_regid++;
			}
		}
	}

	return _regid-1;
	
}

bool AddEdges(IMCOORDINATE seed,vector<IMCOORDINATE> & vec, int _regid)
{
	if (rm.regionID[rm.wid*seed.x +seed.y] == EDGEPT) {
		rm.regionID[rm.wid*seed.x +seed.y] = _regid;
	}
	else if (rm.regionID[rm.wid*seed.x +seed.y] != _regid)
		return false;

	SEGBUF *segbuf = &rm.segbuf[_regid];

	IMCOORDINATE FourNeighbor;

	static int prek=-1;
	static int pren=0;

	int n=0;
	for (int k=0; k<4; k++) 
	{
		double maxerr;
		switch (k) {
		case 0:	FourNeighbor.x = seed.x - 1; FourNeighbor.y = seed.y;maxerr=segbuf->maxDisV; break;
		case 1: FourNeighbor.x = seed.x + 1; FourNeighbor.y = seed.y;maxerr=segbuf->maxDisV; break;
		case 2: FourNeighbor.x = seed.x; FourNeighbor.y = seed.y - 1;maxerr=segbuf->maxDisH; break;
		case 3: FourNeighbor.x = seed.x; FourNeighbor.y = seed.y + 1;maxerr=segbuf->maxDisH; break;
		}
		if (FourNeighbor.x>0&&FourNeighbor.x<rm.len&&
			FourNeighbor.y>0&&FourNeighbor.y<rm.wid&&
			rm.pts[rm.wid*FourNeighbor.x +FourNeighbor.y].i&&
			rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] == EDGEPT)
		{
			double dis, rng;
			dis = ppDistance3fi (&rm.pts[rm.wid*FourNeighbor.x+FourNeighbor.y], &rm.pts[rm.wid*seed.x+seed.y]);
			rng = min(p2r(&rm.pts[rm.wid*FourNeighbor.x+FourNeighbor.y]),p2r(&rm.pts[rm.wid*seed.x+seed.y]));
			if (dis<maxerr) {
				n++;
				if (pren==1 && prek==k) 
					continue;
				vec.push_back(FourNeighbor);
				rm.regionID[rm.wid*FourNeighbor.x +FourNeighbor.y] = _regid;
				prek = k;
			}
		}
	}
	if (!n) prek=-1;
	pren=n;
	return true;
}

void GrowOneEdge(IMCOORDINATE seed, unsigned int _regid)
{
	vector <IMCOORDINATE> Vec1;
	vector <IMCOORDINATE> Vec2;
	bool isVec1 = true;
	int		j;
	IMCOORDINATE	FourNeighbor;
	Vec1.clear();
	Vec2.clear();
	Vec1.push_back(seed);
	while (1)
	{
		switch (isVec1)
		{
		case true:
			Vec2.clear();
			for (j =0; j<Vec1.size(); j++)
			{
				FourNeighbor.x = Vec1[j].x;
				FourNeighbor.y = Vec1[j].y;
				AddEdges(Vec1[j], Vec2, _regid);
			}
			isVec1 = !isVec1;
			if (Vec2.size()<1)
			{
				return;
			}
			break;
		case false:
			Vec1.clear();
			for (j =0; j<Vec2.size(); j++)
			{
				FourNeighbor.x = Vec2[j].x;
				FourNeighbor.y = Vec2[j].y;
				AddEdges(Vec2[j], Vec1, _regid);
			}
			isVec1 = !isVec1;
			if (Vec1.size()<1)
			{
				return;
			}
			break;
		}
	}
}

void EdgeGrow()
{
	IMCOORDINATE	seed;
	int m,x1,x2;
    unsigned int _regid = 1;

	int x,y,xx,yy, k;
	for (x=0; x<rm.len; x++)
	{
		for (y = 0; y<rm.wid; y++)
		{
			if (!rm.pts[rm.wid*x+y].i || rm.regionID[rm.wid*x+y]!=EDGEPT)
				continue;

			bool flg;
			for (k=0; k<4; k++) {
				switch (k) {
				case 0:	xx=x; yy=y-1; flg=true; break;
				case 1:	xx=x-1; yy=y; flg=false; break;
				case 2:	xx=x; yy=y+1; flg=true; break;
				case 3:	xx=x+1; yy=y; flg=false; break;
				}

				if (xx<0 || xx>=rm.len || yy<0 || yy>=rm.wid)
					continue;

				if (rm.regionID[rm.wid*xx+yy]<=0) 
					continue;

				SEGBUF *segbuf = &rm.segbuf[rm.regionID[rm.wid*xx+yy]];
				double maxerr;
				if (flg) maxerr = segbuf->maxDisH;
				else maxerr = segbuf->maxDisV;

				double dis, rng;
				dis = ppDistance3fi (&rm.pts[rm.wid*xx+yy], &rm.pts[rm.wid*x+y]);
				rng = min(p2r(&rm.pts[rm.wid*xx+yy]),p2r(&rm.pts[rm.wid*x+y]));
				if (dis<maxerr) {
					seed.x = x;
					seed.y = y;
					GrowOneEdge(seed, rm.regionID[rm.wid*xx+yy]);
					break;
				}
			}
		}
	}
}


bool ContourSegger()
{
	ContourExtraction();
	
	rm.regnum = RegionGrow()+1;

	Region2Seg ();

	EdgeGrow ();

    return 1;
}

void Region2Seg ()
{
	SEGBUF *segbuf;
	int		regionid;

	rm.segbuf = new SEGBUF[rm.regnum];
	memset (rm.segbuf, 0, sizeof (SEGBUF)*rm.regnum);

	for (regionid=0; regionid<rm.regnum; regionid++) {
		segbuf = &rm.segbuf[regionid];
		segbuf->minDisH = 
		segbuf->minDisV = MAXDISTHRE;
		segbuf->maxDisH = 
		segbuf->maxDisV = 0.0;
	}

	//为每个regionID，生成一个segbuf，记录该区域块的特征
	int x, y;
	for (y=0; y<rm.len; y++) {
		for (x=0; x<rm.wid; x++) {
			if (!rm.pts[y*rm.wid+x].i)
				continue;
			regionid = rm.regionID[y*rm.wid+x];
			if (regionid<=0 || regionid>=rm.regnum) 
				continue;

			segbuf = &rm.segbuf[regionid];

			int xx=x+1, yy=y;
			if (xx<rm.wid && regionid == rm.regionID[yy*rm.wid+xx]) {
				double dis = ppDistance3fi (&rm.pts[y*rm.wid+x], &rm.pts[yy*rm.wid+xx]);
				segbuf->maxDisH = max (segbuf->maxDisH, dis);
				segbuf->minDisH = min (segbuf->minDisH, dis);
			}
			xx=x; yy=y+1;
			if (yy<rm.len && regionid == rm.regionID[yy*rm.wid+xx]) {
				double dis = ppDistance3fi (&rm.pts[y*rm.wid+x], &rm.pts[yy*rm.wid+xx]);
				segbuf->maxDisV = max (segbuf->maxDisV, dis);
				segbuf->minDisV = min (segbuf->minDisV, dis);
			}
		}
	}

	for (regionid=0; regionid<rm.regnum; regionid++) {
		segbuf = &rm.segbuf[regionid];
		if (segbuf->minDisV>segbuf->maxDisV)
			segbuf->maxDisV = 0;
		else
			segbuf->maxDisV = (segbuf->maxDisV+segbuf->minDisV)/2.0;
		if (segbuf->minDisH>segbuf->maxDisH)
			segbuf->maxDisH = 0;
		else
			segbuf->maxDisH = (segbuf->maxDisH+segbuf->minDisH)/2.0;
	}
}

void EstimateSeg ()
{
	SEGBUF *segbuf;
	int		regionid;

	for (regionid=0; regionid<rm.regnum; regionid++) {
		segbuf = &rm.segbuf[regionid];
		segbuf->minp.x =
		segbuf->minp.y =
		segbuf->minp.z = INVALIDDOUBLE;
		segbuf->maxp.x =
		segbuf->maxp.y =
		segbuf->maxp.z = -INVALIDDOUBLE;
		segbuf->minpH.x =
		segbuf->minpH.y =
		segbuf->minpH.z = INVALIDDOUBLE;
		segbuf->maxpH.x =
		segbuf->maxpH.y =
		segbuf->maxpH.z = -INVALIDDOUBLE;
		segbuf->dmin.x = rm.wid;
		segbuf->dmin.y = rm.len;
	}

	//为每个regionID，生成一个segbuf，记录该区域块的特征
	int x, y;
	for (y=0; y<rm.len; y++) {
		for (x=0; x<rm.wid; x++) {
			if (!rm.pts[y*rm.wid+x].i)
				continue;
			regionid = rm.regionID[y*rm.wid+x];
			if (regionid<=0 || regionid>=rm.regnum) 
				continue;

			segbuf = &rm.segbuf[regionid];

			segbuf->dmin.x = min (segbuf->dmin.x, x);
			segbuf->dmin.y = min (segbuf->dmin.y, y);
			segbuf->dmax.x = max (segbuf->dmax.x, x);
			segbuf->dmax.y = max (segbuf->dmax.y, y);

			segbuf->cp.x += rm.pts[y*rm.wid+x].x;
			segbuf->cp.y += rm.pts[y*rm.wid+x].y;
			segbuf->cp.z += rm.pts[y*rm.wid+x].z;

			segbuf->minp.x = min (segbuf->minp.x, double(rm.pts[y*rm.wid+x].x));
			segbuf->minp.y = min (segbuf->minp.y, double(rm.pts[y*rm.wid+x].y));
			segbuf->minp.z = min (segbuf->minp.z, double(rm.pts[y*rm.wid+x].z));
			segbuf->maxp.x = max (segbuf->maxp.x, double(rm.pts[y*rm.wid+x].x));
			segbuf->maxp.y = max (segbuf->maxp.y, double(rm.pts[y*rm.wid+x].y));
			segbuf->maxp.z = max (segbuf->maxp.z, double(rm.pts[y*rm.wid+x].z));
			segbuf->ptnum ++;

			if (rm.pts[y*rm.wid+x].z>1.1) {
				segbuf->minpH.x = min (segbuf->minpH.x, double(rm.pts[y*rm.wid+x].x));
				segbuf->minpH.y = min (segbuf->minpH.y, double(rm.pts[y*rm.wid+x].y));
				segbuf->minpH.z = min (segbuf->minpH.z, double(rm.pts[y*rm.wid+x].z));
				segbuf->maxpH.x = max (segbuf->maxpH.x, double(rm.pts[y*rm.wid+x].x));
				segbuf->maxpH.y = max (segbuf->maxpH.y, double(rm.pts[y*rm.wid+x].y));
				segbuf->maxpH.z = max (segbuf->maxpH.z, double(rm.pts[y*rm.wid+x].z));
				segbuf->ptnumH ++;
			}
		}
	}

	for (regionid=0; regionid<rm.regnum; regionid++) {
		segbuf = &rm.segbuf[regionid];
		if (!segbuf->ptnum)
			continue;
		segbuf->cp.x /= segbuf->ptnum;
		segbuf->cp.y /= segbuf->ptnum;
		segbuf->cp.z /= segbuf->ptnum;
		segbuf->wi = max(0.5,sqrt(sqr(segbuf->maxp.x-segbuf->minp.x)+sqr(segbuf->maxp.y-segbuf->minp.y))); 
	}
}

void ClassiSeg ()
{
	for (int regionid=0; regionid<rm.regnum; regionid++) {
		SEGBUF *segbuf = &rm.segbuf[regionid];
		if (!segbuf->ptnum)
			continue;

		double wiH=0, wi;
		wi = sqrt(sqr(segbuf->maxp.x-segbuf->minp.x)+sqr(segbuf->maxp.y-segbuf->minp.y)); 
		if (segbuf->ptnumH) wiH = sqrt(sqr(segbuf->maxpH.x-segbuf->minpH.x)+sqr(segbuf->maxpH.y-segbuf->minpH.y)); 

		if (segbuf->maxp.z>2.5 || wi>8.0) {			//larger or higher than mv
			if (segbuf->maxp.z>2.0 && wi<2.5)		//tall obj but too thin for a bus 
				segbuf->lab = OBGLAB5;				//could be trunk, pole, sign board etc.
			else if (segbuf->maxp.z<=1.1)			
				segbuf->lab = OBGLAB7;				//low and fat object, could be a part of car/bus or bush
			else
				segbuf->lab = OBGLAB4;				//other large and high object
		}
		else if (segbuf->maxp.z<=1.1) {				//low object
			if (wi<1.0)								//low and small object
				segbuf->lab = OBGLAB6;
			else
				segbuf->lab = OBGLAB7;				//low and fat object, could be a part of car/bus or bush
		}

		//below, maxpH.z in (1.1,2.5] && wi in [0,8.0]

		else if (wi<1.5) {	//wi in [0,1.5)
			if (segbuf->maxpH.z>2.0)				//tall obj, too thin for a bus 
				segbuf->lab = OBGLAB5;
			else {			//maxpH.z in (1.1,2.0]
				if (wiH<1.0) {						//high part is thin
					if (wi>0.2)						//could be a ped
						segbuf->lab = OBGLAB1;
					else
						segbuf->lab = OBGLAB5;		//tall and thin object
				}
				else
					segbuf->lab = OBGLAB3;			//middle height, fat, could be a car
			}
		}
		else if (wi<2.5) {	//wi in [1.5,2.5)
			if (segbuf->maxpH.z>2.0)				//tall obj, too thin for a bus 
				segbuf->lab = OBGLAB5;
			else {			//maxpH.z in (1.1,2.0]
				if (wiH<1.0) 						//high part is thin
					segbuf->lab = OBGLAB2;			//could be a bicycle
				else
					segbuf->lab = OBGLAB3;			//middle height, fat, could be a car
			}
		}
		else {	//wi in [2.5,8.0]
			segbuf->lab = OBGLAB3;
		}
	}
}
