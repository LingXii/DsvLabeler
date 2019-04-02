#include "DsvLabelMv.h"

//dm立体栅格的范围和像素大小
double	MAXRANGEDIST = 80.0;
double	MINHEIGHT = 0.0;
double	MAXHEIGHT = 3.0;
DMAP	dm;

void InitDmap ()
{
	dm.ox = -MAXRANGEDIST;
	dm.oy = -MAXRANGEDIST;
	dm.oz = MINHEIGHT;
	dm.h_pixelsize = 0.3;		//横向像素大小
	dm.v_pixelsize = 0.5;		//纵向像素大小
	dm.wid = int(MAXRANGEDIST*2.0/dm.h_pixelsize);
	dm.len = int(MAXRANGEDIST*2.0/dm.h_pixelsize);
	dm.hei = int((MAXHEIGHT-MINHEIGHT)/dm.v_pixelsize);
	dm.cells = new ONEDCELL[dm.wid*dm.len*dm.hei];
	dm.val = false;
}

void ReleaseDmap ()
{
	if (dm.val && dm.cells)
		delete []dm.cells;
}

// round = 0: 仅关联同一位置上的数据
// round = 1: 关联同一位置及相邻位置上的数据
ONEDCELL *GetOneRid (double x, double y, double z, double wi, double mz, int round)
{
	if (!dm.val)
		return 0;

	int i = nint((x-dm.ox)/dm.h_pixelsize);
	int j = nint((y-dm.oy)/dm.h_pixelsize);
	int k = nint((z-dm.oz)/dm.v_pixelsize);

	double dwi1,dwz1,dwi2,dwz2;

	if (i<0 || i>=dm.wid || j<0 || j>=dm.len || k<0 || k>=dm.hei)
		return NULL;
	if (dm.cells[k*dm.wid*dm.len+j*dm.wid+i].num<0)
		return NULL;
	else if (dm.cells[k*dm.wid*dm.len+j*dm.wid+i].num>0) {
		dwi1 = fabs(dm.cells[k*dm.wid*dm.len+j*dm.wid+i].wi-wi)/wi;
		dwz1 = fabs(dm.cells[k*dm.wid*dm.len+j*dm.wid+i].mz-mz)/mz;
		dwi2 = fabs(dm.cells[k*dm.wid*dm.len+j*dm.wid+i].wi-wi)/dm.cells[k*dm.wid*dm.len+j*dm.wid+i].wi;
		dwz2 = fabs(dm.cells[k*dm.wid*dm.len+j*dm.wid+i].mz-mz)/dm.cells[k*dm.wid*dm.len+j*dm.wid+i].mz;
		if (dwi1<1.0 && dwz1<0.5 && dwi2<1.0 && dwz2<0.5)
			return &dm.cells[k*dm.wid*dm.len+j*dm.wid+i];
		else
			return NULL;
	}

	if (!round)
		return NULL;

	int count = 0;
	int prid, pi, pj;
	for (int jr=-1; jr<=1; jr++) {
		for (int ir=-1; ir<=1; ir++) {
			if ((abs(jr)+abs(ir))!=1)
				continue;
			int ii = i+ir;
			int jj = j+jr;

			if (ii<0 || ii>=dm.wid || jj<0 || jj>=dm.len || k<0 || k>=dm.hei)
				return NULL;
			if (dm.cells[k*dm.wid*dm.len+jj*dm.wid+ii].num<=0) 
				continue;

			if (!count) {
				prid = dm.cells[k*dm.wid*dm.len+jj*dm.wid+ii].rid;
				pi = ii;
				pj = jj;
				count ++;
			}
			else {
				if (prid != dm.cells[k*dm.wid*dm.len+jj*dm.wid+ii].rid) 
					return NULL;
			}
		}
	}

	if (count) {
		dwi1 = fabs(dm.cells[k*dm.wid*dm.len+pj*dm.wid+pi].wi-wi)/wi;
		dwz1 = fabs(dm.cells[k*dm.wid*dm.len+pj*dm.wid+pi].mz-mz)/mz;
		dwi2 = fabs(dm.cells[k*dm.wid*dm.len+pj*dm.wid+pi].wi-wi)/dm.cells[k*dm.wid*dm.len+pj*dm.wid+pi].wi;
		dwz2 = fabs(dm.cells[k*dm.wid*dm.len+pj*dm.wid+pi].mz-mz)/dm.cells[k*dm.wid*dm.len+pj*dm.wid+pi].mz;
		if (dwi1<1.0 && dwz1<0.5 && dwi2<1.0 && dwz2<0.5)
			return &dm.cells[k*dm.wid*dm.len+pj*dm.wid+pi];
	}
	return NULL;
}

void SetOneRid (double x, double y, double z, int rid, double wi, double mz)
{
	int i = nint((x-dm.ox)/dm.h_pixelsize);
	int j = nint((y-dm.oy)/dm.h_pixelsize);
	int k = nint((z-dm.oz)/dm.v_pixelsize);

	if (i<0 || i>=dm.wid || j<0 || j>=dm.len || k<0 || k>=dm.hei)
		return;
	if (!dm.cells[k*dm.wid*dm.len+j*dm.wid+i].num) {
		dm.cells[k*dm.wid*dm.len+j*dm.wid+i].rid = rid;
		dm.cells[k*dm.wid*dm.len+j*dm.wid+i].wi = wi;
		dm.cells[k*dm.wid*dm.len+j*dm.wid+i].mz = mz;
		dm.cells[k*dm.wid*dm.len+j*dm.wid+i].num ++;
	}
	else
	if (dm.cells[k*dm.wid*dm.len+j*dm.wid+i].num>0) {
		if (dm.cells[k*dm.wid*dm.len+j*dm.wid+i].rid != rid)
			dm.cells[k*dm.wid*dm.len+j*dm.wid+i].num = -1;
	}
}

//dm是一个立体栅格，记录的是前一帧传感器坐标系下的regionID
//对每个立体栅格，如果拥有不同regionID的点投影到该栅格，则该栅格为无效，num设置为-1
//每一帧处理中，对每个激光点进行坐标转换，转换到前一帧传感器坐标系，查找regionID（rid），如果找到，则关联数据

typedef struct {
	int				prid;
	int				num;
} PridCell;

int PridCellSort (PridCell t1, PridCell t2)
{
	return (t1.num>t2.num);
}

void DataAssociation ()
{
	SEGBUF *segbuf;
	int		regionid;
	extern FILE	*seglogfp;
	extern int	dFrmNo;

	//准备坐标转换矩阵
	MATRIX	Rot, pRot;
	point3d	shv;
	if (dm.val) {
		createRotMatrix_ZYX(Rot, 0, 0, rm.ang.z-dm.ang.z ) ;			//Rot = R_c*^{-1}*R_{t-1}^{-1}*R_t*R_c = R_{t-1}^{-1}*R_t
		createRotMatrix_ZYX(pRot, 0, 0, -calibInfo.ang.z-dm.ang.z ) ;	//pRot = R_c*^{-1}*R_{t-1}^{-1}
		shv.x = rm.shv.x-dm.shv.x;										//shv = Shv_t-shv_{t-1}
		shv.y = rm.shv.y-dm.shv.y;
		shv.z = 0.0;
		rotatePoint3d(shv, pRot);										//shv = pRot*shv

		vector <PridCell> pcell;
		PridCell	pc;

		vector <ONEDCELL> dc;

		point3fi	pi, pii;
		point3d		pt;
		int			xi, yi;

		for (int round=0; round<2; round++) {
			for (regionid=0; regionid<rm.regnum; regionid++) {
				segbuf = &rm.segbuf[regionid];
				if (!segbuf->ptnum)
					continue;
				if (segbuf->prid)
					continue;

				//查找区域块的中心点，赋予x0和y0

				pcell.clear();

				int tot=0;
				for (yi=segbuf->dmin.y; yi<=segbuf->dmax.y; yi++) {
					for (xi=segbuf->dmin.x; xi<=segbuf->dmax.x; xi++) {
						if (rm.pts[yi*rm.wid+xi].i && rm.regionID[yi*rm.wid+xi]==regionid) 
							break;
					}
					if (xi>segbuf->dmax.x)
						continue;
					for (; xi<=segbuf->dmax.x; xi++) {
						if (rm.pts[yi*rm.wid+xi].i && rm.regionID[yi*rm.wid+xi]!=regionid) 
							break;

						tot++;
						pt.x = rm.pts[yi*rm.wid+xi].x;
						pt.y = rm.pts[yi*rm.wid+xi].y;
						pt.z = rm.pts[yi*rm.wid+xi].z;

						//将pt从当前传感器坐标系转换到上一时刻传感器坐标系
						//激光点均已矫正到以地面为XY面的坐标系
						rotatePoint3d(pt, Rot);
						shiftPoint3d(pt, shv);
				
						//仅查找同一位置上的数据包
						ONEDCELL *dc = GetOneRid (pt.x, pt.y, pt.z, segbuf->wi, segbuf->maxp.z, round);
						if (!dc || !dc->rid)
							continue;
					
						pc.prid = dc->rid;
						dc->num = -1;
						if (pc.prid) {
							int n;
							for (n=0; n<pcell.size(); n++) {
								if (pcell[n].prid==pc.prid) {
									pcell[n].num ++;
									break;
								}
							}
							if (n>=pcell.size()) {
								pc.num = 1;
								pcell.push_back(pc);
							}
						}
					}
				}
				if (!pcell.size())
					continue;
				sort (pcell.begin(), pcell.end(), PridCellSort);  
				//minimal overlapping ratio = 0.1
//				if ((double)pcell.begin()->num/(double)tot>0.1)
					segbuf->prid = pcell.begin()->prid;
			}
		}

		for (regionid=0; regionid<rm.regnum; regionid++) {
			segbuf = &rm.segbuf[regionid];
			if (!segbuf->ptnum)
				continue;
			if (!segbuf->prid)
				segbuf->prid = (dFrmNo+1)*10000+regionid;

			int y0=(segbuf->dmin.y+segbuf->dmax.y)/2;
			double mindis=100.0;
			for (xi=segbuf->dmin.x; xi<=segbuf->dmax.x; xi++) {
				if (rm.pts[y0*rm.wid+xi].i && rm.regionID[y0*rm.wid+xi]==regionid) {
                    mindis = sqrt(SQR(rm.pts[y0*rm.wid+xi].x)+SQR(rm.pts[y0*rm.wid+xi].y)+SQR(rm.pts[y0*rm.wid+xi].z));
					break;
				}
			}
			for (xi=segbuf->dmax.x; xi>=segbuf->dmin.x; xi--) {
				if (rm.pts[y0*rm.wid+xi].i && rm.regionID[y0*rm.wid+xi]==regionid) {
                    mindis = min(mindis,(double)sqrt(SQR(rm.pts[y0*rm.wid+xi].x)+SQR(rm.pts[y0*rm.wid+xi].y)+SQR(rm.pts[y0*rm.wid+xi].z)));
					break;
				}
			}

			int x0=(segbuf->dmin.x+segbuf->dmax.x)/2;
			int dx = max (x0, rm.wid-x0-1);
			bool cont=true;
			for (int d=0; d<=dx; d++) {
				for (int k=0; k<2; k++) {
					if (k) 
						xi = x0+d;
					else
						xi = x0-d;
					if (xi>=0 && xi<rm.wid) {
						if (rm.pts[y0*rm.wid+xi].i && rm.regionID[y0*rm.wid+xi]==regionid) {
							fprintf (seglogfp, "rno=%d\n", regionid);
//							if (segbuf->prid) {
								fprintf (seglogfp, "sno=%d,x=%.3f,y=%.3f,z=%.3f,col=%d,row=%d,prid=%d,pnum=%d,mind=%.3f\n", 
									0,rm.pts[y0*rm.wid+xi].x,rm.pts[y0*rm.wid+xi].y,rm.pts[y0*rm.wid+xi].z,
									xi, y0,segbuf->prid,segbuf->ptnum,mindis);
/*							}
							else {
								fprintf (seglogfp, "sno=%d,x=%.3f,y=%.3f,z=%.3f,col=%d,row=%d,prid=%d,pnum=%d,mind=%.3f\n", 
									0,rm.pts[y0*rm.wid+xi].x,rm.pts[y0*rm.wid+xi].y,rm.pts[y0*rm.wid+xi].z,
									xi, y0,(dFrmNo+1)*10000+regionid,segbuf->ptnum,mindis);
							}
*/							cont=false;
							break;
						}
					}
				}
				if (!cont)
					break;
			}
		}
	}

	//将当前帧的rid写入dm
	memset (dm.cells, 0, sizeof(ONEDCELL)*dm.wid*dm.len*dm.hei);
	dm.ang = rm.ang;
	dm.shv = rm.shv;
	for (int y=0; y<rm.len; y++) {
		for (int x=0; x<rm.wid; x++) {
			if (!rm.pts[y*rm.wid+x].i)
				continue;
			regionid = rm.regionID[y*rm.wid+x];
			if (regionid<=0 || regionid>=rm.regnum) 
				continue;

			segbuf = &rm.segbuf[regionid];

			if (segbuf->prid)
				SetOneRid (rm.pts[y*rm.wid+x].x, rm.pts[y*rm.wid+x].y, rm.pts[y*rm.wid+x].z, segbuf->prid, segbuf->wi, segbuf->maxp.z);
			else {
				int prid = (dFrmNo+1)*10000+regionid;
				SetOneRid (rm.pts[y*rm.wid+x].x, rm.pts[y*rm.wid+x].y, rm.pts[y*rm.wid+x].z, prid, segbuf->wi, segbuf->maxp.z);
			}
		}
	}
	dm.val = true;
}

