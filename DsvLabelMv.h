#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int maxn = 2000000000;
const double topi = acos(-1.0)/180.0;	// pi/180
#define BOUND(x,min,max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define	nint(x)			(int)((x>0)?(x+0.5):(x-0.5))
#define	sqr(x)			((x)*(x))

struct point2d
{
	double x;
	double y;
};

struct point3d
{
    double x;
    double y;
    double z;
};

typedef double  MATRIX[3][3] ; 

typedef struct {
	point3d			ang;
	point3d			shv;
	MATRIX			rot;
} TRANSINFO;

#define	PNTS_PER_LINE		32
#define	LINES_PER_BLK		12
#define	PTNUM_PER_BLK		(32*12)
#define	BKNUM_PER_FRM		180
#define	SCANDATASIZE		(180*12)

//HORIERRFACTOR=tan（水平角分辨率=0.167度）*（放大系数=2.0）=0.0029*2.0
#define	HORIERRFACTOR	0.02	//0.006
//VERTERRFACTOR=tan（垂直角分辨率=1.323度）*（放大系数=1.5）=0.023*1.5
#define	VERTERRFACTOR	0.05	//0.035
#define	BASEERROR		0.1
#define	MAXSMOOTHERR	1.0
#define	MAXDISTHRE		2.0

#define	INVALIDDOUBLE		99999999.9


typedef struct {
	float			x, y, z;
	u_char			i;
} point3fi;

typedef struct {
	int x, y;
} point2i;

typedef struct {
    int			millisec;
	point3fi		points[PTNUM_PER_BLK];
} ONEVDNDATA;

typedef struct {
	point3d			ang;
	point3d			shv;
    int			millisec;
	point3fi		points[PTNUM_PER_BLK];
	int				lab[PTNUM_PER_BLK];
} ONEDSVDATA;

typedef struct {
	ONEDSVDATA		dsv[BKNUM_PER_FRM];
	point3d			ang;
	point3d			shv;
	MATRIX			rot;
} ONEDSVFRAME;

typedef struct {
	int				rid;
	double			wi;
	double			mz;
	int				num;
} ONEDCELL;

typedef struct {
	int				wid;
	int				len;
	int				hei;
	double			h_pixelsize;
	double			v_pixelsize;
	double			ox;
	double			oy;
	double			oz;
	ONEDCELL		*cells;
	bool			val;
	point3d			ang;
	point3d			shv;
} DMAP;

typedef	struct {
	unsigned short	lab;// label
	int			prid;
	point2i		dmin;
	point2i		dmax;   // a bounding box which contain the region in range image
	point3d		maxp;
	point3d		minp;   // a 3D bounding box which contain the region in space
	int			ptnum;  // number of points
	point3d		maxpH;
	point3d		minpH;  // a 3D bounding box which contain the high part of region in space
	int			ptnumH; // number of points of the high part
	point3d		cp;     // center of the region
	double		wi;     // length of diagonal of the cross section
	double		minDisH;// minimum space distance of horizontal neighbour pixels
	double		minDisV;// minimum space distance of vertical neighbour pixels
	double		maxDisH;
	double		maxDisV;
} SEGBUF;

typedef struct {
	int				wid;
	int				len;
	point3fi		*pts;
    unsigned char			*segflg;
	int				*regionID;
	int				regnum;
	SEGBUF			*segbuf;
	IplImage		*rMap;
	IplImage		*lMap;
	IplImage		*dMap;
	IplImage		*sMap;
	point3d			ang;
	point3d			shv;
} RMAP;

#define	INVAFLG			0
#define	LINEFLG			10
#define	NODEFLG			11
#define	SEGFLG			12
#define	ROADFLG			13
#define	HIGHFLG			14
#define	LOHOFLG			15

#define	OBGLAB0			0
#define	OBGLAB1			1
#define	OBGLAB2			2
#define	OBGLAB3			3
#define	OBGLAB4			4
#define	OBGLAB5			5
#define	OBGLAB6			6
#define	OBGLAB7			7
#define OBJGROUND		100
#define	OBJBACKGROUND	101


#define UNKNOWN			0
#define NONVALID		-9999
#define GROUND			-999
#define	BACKGROUND		-99
#define EDGEPT			-9
#define	CHECKEDLABEL	-7

typedef struct {
    int		milli;
	point2d		gp;
	point2d		lp;
	double		len1, len2;
} TRAJPT;

typedef struct {
	TRAJPT		*trjs;
	int			trjnum;
	int			type;
	int			curpr;
} ONEMVOBJ;

typedef struct {
	ONEMVOBJ		*mvobjs;
	int				mvobjnum;
    bool			loaded;
} MVOBJS;

extern RMAP	rm;
extern TRANSINFO calibInfo;

void rMatrixInit (MATRIX &rt);
void rMatrixmulti (MATRIX &r, MATRIX &rt);
void createRotMatrix_ZYX (MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void createRotMatrix_XYZ (MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void createRotMatrix_ZXY (MATRIX &rt, double rotateX, double rotateY, double rotateZ);
void shiftPoint3d (point3d &pt, point3d &sh);
void rotatePoint3d (point3d &pt, MATRIX &a);
double normalize2d (point2d *p);
double ppDistance2d (point2d *p1, point2d *p2);
double innerProduct2d (point2d *v1, point2d *v2);
double ppDistance3fi (point3fi *pt1, point3fi *pt2);
double p2r (point3fi *pt1);
void rotatePoint3fi (point3fi &pt, MATRIX &a);

ONEMVOBJ *GetOneMvObj (int pos);
bool LoadTrajData (char *filename);
void SegmentScanLine (point3fi *pts, unsigned char *segflg);
bool ContourSegger();
void Calculate_Plane(int Points_Total, double *X_Coord, double *Y_Coord, double *Z_Coord,
					 int Origin_Flag, double Plane_Eq[4]);
void Calculate_Residuals(double *X, double *Y, double *Z, double Equation[4], 
						 double *Error, int PointsTotal);
bool EstimateGround (double Equation[], double &Error);
void CorrectPoints (double Equation[]);
void LabelBackground ();
void SmoothingData ();
void Region2Seg ();
void EstimateSeg ();
void ContourExtraction();
unsigned int RegionGrow();
void EdgeGrow();
void ClassiSeg ();
void OutputLog (char *filename, char *str);
void InitDmap ();
void ReleaseDmap ();
void DataAssociation ();

void INVshiftPoint3d (point3d &pt, point3d &sh);
void INVrotatePoint3d (point3d &pt, MATRIX &a);
