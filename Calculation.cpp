
#include "DsvLabelMv.h"

void rMatrixInit (MATRIX &rt)
{
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;
}

/////////////////////////////////////////////////////////////////////////////
//
//  機能     : ３＊３行列の乗算
//
//  入力     : r		３＊３行列
//  　　       rt		３＊３行列
//  
//  返り値   : r <-	r ＊ rt
//  
//  備考     : 呼び出しファンクション 
//　		　 なし
//  
/////////////////////////////////////////////////////////////////////////////

void rMatrixmulti (MATRIX &r, MATRIX &rt)
{
	double	rin[3][3];	 
	int		i, j;

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			rin[i][j] = r[i][j];

	for (i=0; i<3; i++) 
		for (j=0; j<3; j++) { 
			r[i][j] = rin[i][0]*rt[0][j] +
					  rin[i][1]*rt[1][j] +
					  rin[i][2]*rt[2][j];
		}
}

/////////////////////////////////////////////////////////////////////////////
//
//  機能     : 回転角度により３＊３座標系変換行列を求める
//
//  入力     : rt		３＊３座標系変換行列
//  　　       rotateX	Ｘ軸の回転角（ロー角）   単位：
//  　　       rotateY	Ｙ軸の回転角（ピーチ角）
//  　　       rotateZ	Ｚ軸の回転角（ヨー角）
//  
//  返り値   : rt <- Rz * Ry * Rx
//  
//  備考     : 呼び出しファンクション 
//　		　 �@ rMatrixmulti
//  
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_ZYX (MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;

	if (rotateZ!=0.0) {
		/*	R3 :   cosz  -sinz   0.0
				   sinz  cosz   0.0
				   0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti (rt, rr);
	}

	if (rotateY!=0.0) {
		/*	R2 :   cosy   0.0  siny
			 		0.0   1.0   0.0
				  -siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti (rt, rr);
	}

	if (rotateX!=0.0) {
		/*	R1 :	1.0   0.0   0.0
					0.0  cosx  -sinx
					0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti (rt, rr);
	}
}

/////////////////////////////////////////////////////////////////////////////
//
//  機能     : 回転角度により３＊３座標系変換行列を求める
//
//  入力     : rt		３＊３座標系変換行列
//  　　       rotateX	Ｘ軸の回転角（ロー角）   単位：
//  　　       rotateY	Ｙ軸の回転角（ピーチ角）
//  　　       rotateZ	Ｚ軸の回転角（ヨー角）
//  
//  返り値   : rt <- Rz * Rx * Ry
//  
//  備考     : 呼び出しファンクション 
//　		　 �@ rMatrixmulti
//  
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_ZXY (MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;

	if (rotateZ!=0.0) {
		/*	R3 :   cosz  -sinz   0.0
				   sinz  cosz   0.0
				   0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti (rt, rr);
	}


	if (rotateX!=0.0) {
		/*	R1 :	1.0   0.0   0.0
					0.0  cosx  -sinx
					0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti (rt, rr);
	}

	if (rotateY!=0.0) {
		/*	R2 :   cosy   0.0  siny
			 		0.0   1.0   0.0
				  -siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti (rt, rr);
	}

}

/////////////////////////////////////////////////////////////////////////////
//
//  機能     : 回転角度により３＊３座標系変換行列を求める
//
//  入力     : rt		３＊３座標系変換行列
//  　　       rotateX	Ｘ軸の回転角（ロー角）   単位：
//  　　       rotateY	Ｙ軸の回転角（ピーチ角）
//  　　       rotateZ	Ｚ軸の回転角（ヨー角）
//  
//  返り値   : rt <- Rx * Ry * Rz
//  
//  備考     : 呼び出しファンクション 
//　		　 �@ rMatrixmulti
//  
/////////////////////////////////////////////////////////////////////////////

void createRotMatrix_XYZ (MATRIX &rt, double rotateX, double rotateY, double rotateZ)
{
	double	sinx, siny, sinz, cosx, cosy, cosz;
	double	rr[3][3];
	int		i, j;

	sinx = sin(rotateX);
	siny = sin(rotateY);
	sinz = sin(rotateZ);
	cosx = cos(rotateX);
	cosy = cos(rotateY);
	cosz = cos(rotateZ);

	for (i=0; i<3; i++)
		for (j=0; j<3; j++)
			if (i==j) rt[i][j] = 1;
			else rt[i][j] = 0;

	if (rotateX!=0.0) {
		/*	R1 :	1.0   0.0   0.0
					0.0  cosx  -sinx
					0.0  sinx  cosx
		*/
		rr[0][0] = 1.0;
		rr[0][1] = 0.0;
		rr[0][2] = 0.0;
		rr[1][0] = 0.0;
		rr[1][1] = cosx;
		rr[1][2] = -sinx;
		rr[2][0] = 0.0;
		rr[2][1] = sinx;
		rr[2][2] = cosx;
		rMatrixmulti (rt, rr);
	}

	if (rotateY!=0.0) {
		/*	R2 :   cosy   0.0 siny
			 		0.0   1.0   0.0
				   -siny   0.0  cosy
		*/
		rr[0][0] = cosy;
		rr[0][1] = 0.0;
		rr[0][2] = siny;
		rr[1][0] = 0.0;
		rr[1][1] = 1.0;
		rr[1][2] = 0.0;
		rr[2][0] = -siny;
		rr[2][1] = 0.0;
		rr[2][2] = cosy;
		rMatrixmulti (rt, rr);
	}

	if (rotateZ!=0.0) {
		/*	R3 :   cosz  -sinz   0.0
				   sinz cosz   0.0
					0.0   0.0   1.0
		*/
		rr[0][0] = cosz;
		rr[0][1] = -sinz;
		rr[0][2] = 0.0;
		rr[1][0] = sinz;
		rr[1][1] = cosz;
		rr[1][2] = 0.0;
		rr[2][0] = 0.0;
		rr[2][1] = 0.0;
		rr[2][2] = 1.0;
		rMatrixmulti (rt, rr);
	}
}


void shiftPoint3d (point3d &pt, point3d &sh)
{
	point3d		p;

	p.x = pt.x + sh.x;
	p.y = pt.y + sh.y;
	p.z = pt.z + sh.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void rotatePoint3d (point3d &pt, MATRIX &a)
{
	point3d	p;

	p.x = a[0][0]*pt.x + a[0][1]*pt.y + a[0][2]*pt.z;
	p.y = a[1][0]*pt.x + a[1][1]*pt.y + a[1][2]*pt.z;
	p.z = a[2][0]*pt.x + a[2][1]*pt.y + a[2][2]*pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void INVshiftPoint3d (point3d &pt, point3d &sh)
{
	pt.x = pt.x - sh.x;
	pt.y = pt.y - sh.y;
	pt.z = pt.z - sh.z;
}

void INVrotatePoint3d (point3d &pt, MATRIX &a)
{
	point3d	p;

	p.x = a[0][0]*pt.x + a[1][0]*pt.y + a[2][0]*pt.z;
	p.y = a[0][1]*pt.x + a[1][1]*pt.y + a[2][1]*pt.z;
	p.z = a[0][2]*pt.x + a[1][2]*pt.y + a[2][2]*pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}

void rotatePoint3fi (point3fi &pt, MATRIX &a)
{
	point3d	p;

	p.x = a[0][0]*pt.x + a[0][1]*pt.y + a[0][2]*pt.z;
	p.y = a[1][0]*pt.x + a[1][1]*pt.y + a[1][2]*pt.z;
	p.z = a[2][0]*pt.x + a[2][1]*pt.y + a[2][2]*pt.z;
	pt.x = p.x;
	pt.y = p.y;
	pt.z = p.z;
}


double ppDistance2d (point2d *p1, point2d *p2)
{
    return sqrt(SQR(p1->x-p2->x)+SQR(p1->y-p2->y));
}

double innerProduct2d (point2d *v1, point2d *v2)
{
	return (v1->x*v2->x+v1->y*v2->y);
}

double normVector2d (point2d *p)
{
    return (sqrt(SQR(p->x)+SQR(p->y)));
}

double normalize2d (point2d *p)
{
	const   double NORM_EPS = 1.0e-10;
	double	norm;

	norm = normVector2d (p);
	if (norm) {
		p->x /= (norm + NORM_EPS);
		p->y /= (norm + NORM_EPS);
	}
	return norm;
}

double ppDistance3fi (point3fi *pt1, point3fi *pt2)
{
    return (sqrt(SQR(pt1->x-pt2->x)+SQR(pt1->y-pt2->y)+SQR(pt1->z-pt2->z)));
}

double p2r (point3fi *pt1)
{
    return (sqrt(SQR(pt1->x)+SQR(pt1->y)+SQR(pt1->z)));
}
