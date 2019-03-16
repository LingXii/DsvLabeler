
#include "DsvLabelMv.h"
#include "LineSegger.h"

#define	MINLINEPT			3
#define MIN_DEV				0.1   /* minimum allowable deviation  ad hoc value! */
#define	MAXBLANKCNT			4


/////////////////////////////////////////////////////////////////////////////
//
//	THE FOLLOWING PROGRAMS WERE ORIGINALLY DEVELOPED BY 
//		A  P.L. Rosin / G.A.W.West production. Copyright 1987 / 1988.
//getOnelist ()
//segment ()
//transform ()
//improve_lines ()
//  
/////////////////////////////////////////////////////////////////////////////

#define NINETY 1.570796327

#define OK 0
#define NULL_BRKPT	-3
#define	NULL_DEV	-6
#define NULL_SIG	-9999

/* one list of pixels */
double x_c[SCANDATASIZE],y_c[SCANDATASIZE];
double sigs[SCANDATASIZE],sums[SCANDATASIZE];
int flags[SCANDATASIZE];

/* x_c2/y_c2 transformed and rotated */
double x_c2[SCANDATASIZE],y_c2[SCANDATASIZE];
int end2;        /* end of segments in x_c2/y_c2 */

double min_dev = MIN_DEV;	/*  minimum deviation for a line - default MIN_DEV but can
							be set by user */

void SegmentScanLine (point3fi *pts, unsigned char *segflg)
{
	int				i, j, s, startid, endid, cnt;
	double			rangeDis;
	double			sig, sum;

	for (i=0; i<SCANDATASIZE; i++) {
		sigs[i] = NULL_SIG;
		flags[i] = OK;
		if (pts[i].i) {
			segflg[i] = SEGFLG;
			x_c[i] = pts[i].x;
			y_c[i] = pts[i].y;
		}
		else {
			segflg[i] = INVAFLG;
			x_c[i] = y_c[i] = INVALIDDOUBLE;
		}
	}

	startid = 0;

	while (getOnelist (&startid, &endid)) {

		segment(startid, endid,&sig,&sum);
		i=startid;
		while (i<=endid) {
			if (flags[i] != NULL_BRKPT) {
				cnt = 0;
				j = i;
				do {
					if (pts[i].i)
						cnt ++;
					i ++;
				} while ((i<=endid)&&(flags[i]==NULL_BRKPT));

				if (cnt<=MINLINEPT) {
					for (s=j+1; s<=i-1; s++) {
						flags[s] = OK;
					}
				}
			}
		}

		improve_lines(startid, endid);

		i=startid;
		while (i<=endid) {
			cnt = 0;
			j = i;
			do {
				if (pts[i].i)
					cnt ++;
				i ++;
			} while ((i<=endid)&&(flags[i]==NULL_BRKPT));

			if (cnt<=MINLINEPT) continue;

			for (s=j+1; s<i; s++) {
				if (segflg[s] == SEGFLG)
					segflg[s] = LINEFLG;
			}
			segflg[j] = segflg[i] = NODEFLG;

		}
		startid = ++endid;
	}
}

bool getOnelist (int *startid, int *endid)
{
	int				i, ipre;
	point2d			ptpre, pt;
	double			dis, dislim, rangeDis;
    bool			ret = false;

	i=*startid;

	while (!ret) {

		for (; i<SCANDATASIZE; i++) {
			if ((x_c[i]!=INVALIDDOUBLE)||(y_c[i]!=INVALIDDOUBLE))
				break;
		}
		if (i>=SCANDATASIZE)
            return false;

		ptpre.x = x_c[i];
		ptpre.y = y_c[i];
		ipre = i;

		*startid = i++;
		for (; i<SCANDATASIZE; i++) {
			if ((i-ipre)>MAXBLANKCNT) {
				break;
			}
			if ((x_c[i]!=INVALIDDOUBLE)||(y_c[i]!=INVALIDDOUBLE)) {
				pt.x = x_c[i];
				pt.y = y_c[i];
				rangeDis = sqrt(sqr(pt.x)+sqr(pt.y));
				dis = sqrt(sqr(pt.x-ptpre.x)+sqr(pt.y-ptpre.y));
				dislim = max(0.5,tan(0.25*(double)(i-ipre)*M_PI/180.0)*rangeDis);
				if (dis>dislim) {
					break;
				}
				ptpre.x = pt.x;
				ptpre.y = pt.y;
				ipre = i;
				*endid = ipre;
                ret = true;
			}
		}
	}
    return true;
}

void segment(int start_in, int finish_in, double *sig_out, double *sum_out)
{
    int i;
    int pos;
    double dev;
    double sig1,sig2,sig3,max_sig,sum1,sum2,sum3,best_sum;
    int ok;
    double length;

    /* compute significance at this level */

    transform(start_in,finish_in);
    deviation(&pos,&dev,&ok,&sum1);

    if (dev == 0)
       dev = min_dev;

    pos = pos + start_in - 1;
    /* euclidean length */
    length = sqr(x_c2[1]-x_c2[end2]);
    length += sqr(y_c2[1]-y_c2[end2]);
    length = sqrt(length);
//    sig1 = dev / length;
	sig1 = dev;
    sum1 = sum1 / length;
	if ((finish_in - start_in) < 3) {
		;
	}
    else if (((finish_in - start_in) < 3) || (dev < min_dev) || (ok == false)) {
        /* save line match at this lowest of levels */
        /* modify x_c,y_c data to delete unused coordinates */
        /* save significance */
        sigs[start_in] = sig1;
        sums[start_in] = sum1;
        *sig_out = sig1;
        *sum_out = sum1;
        /* delete breakpoints between end points */
        if ((finish_in - start_in) >= 2)
            for (i = start_in + 1; i < finish_in;i++)
                flags[i] = NULL_BRKPT;
    }
    else{
        /* recurse to next level down */
        segment(start_in,pos,&sig2,&sum2);
        segment(pos,finish_in,&sig3,&sum3);

        /* get best significance from lower level */
        if (sig2 < sig3) {
            max_sig = sig2;
            best_sum = sum2;
        }
        else{
            max_sig = sig3;
            best_sum = sum3;
        }
        if (max_sig < sig1) {
            /* return best significance, keep lower level description */
            *sig_out = max_sig;
            *sum_out = best_sum;
        }
        else{
            /* line at this level is more significant so remove coords
               at lower levels */
            *sig_out = sig1;
            *sum_out = sum1;
            sigs[start_in] = *sig_out;
            sums[start_in] = *sum_out;
            if ((finish_in - start_in) >= 2)
                for (i = start_in + 1;i < finish_in;i++)
                    flags[i] = NULL_DEV;
        }
    }
}

void transform (int start,int finish)
{
    int i,j;
    double x_offset,y_offset,x_end,y_end;
    double angle,sine,cosine;
    double temp;

    x_offset = x_c[start];
    y_offset = y_c[start];
    x_end = x_c[finish];
    y_end = y_c[finish];
    if ((x_end - x_offset) == 0.0) {
        if ((y_end - y_offset) > 0.0)
            angle = -NINETY;
        else
            angle = NINETY;
    }
    else{
        temp = ((double)(y_end-y_offset) / (double)(x_end-x_offset));
        angle = -atan(temp);
    }
    cosine = cos(angle);
    sine = sin(angle);
    j = 0;
    for (i=start;i<=finish;i++) {
        j++;
        x_c2[j] = x_c[i] - x_offset;
        y_c2[j] = y_c[i] - y_offset;
        temp = (double)(cosine * x_c2[j]) - (double)(sine * y_c2[j]);
        y_c2[j] = (double)(sine * x_c2[j]) + (double)(cosine * y_c2[j]);
        x_c2[j] = temp;
    }
    end2 = j;
}

void deviation (int *pos, double *dev, int *ok, double *sum)
{
    int i;
    int pos1;
    double max1,temp;  /* temp used for abs deviation - dont change!! */

    pos1 = 0;
    max1 = 0.0;
    *sum = 0.0;
    for (i = 1;i <= end2;i++) {
       temp = fabs(y_c2[i]);
       if (temp > max1) {
          max1 = temp;
          pos1 = i;
       }
       *sum += temp;
    }
    /* if no peak found - signal with ok */
    if (max1 == 0.0)
       *ok = false;
    else
        *ok = true;
    *pos = pos1;
    *dev = max1;
}

void improve_lines (int start_in,int finish_in)
{

    int i, j, ii, jj, iii;
    int pos;
    double dev;
    double sum1;
    int ok;

	i=start_in;
	
	while (i<finish_in) {
		iii = i;
		for (; i<finish_in; i++) {
			if (flags[i] == NULL_BRKPT)
				break;
		}
		if (i>=finish_in)
			break;
		i --;
		for (j=i+1; j<finish_in; j++) {
			if (flags[j] != NULL_BRKPT)
				break;
		}

		for (ii=i-1; ii>start_in; ii--) {
			if (flags[ii-1] == NULL_BRKPT)
				break;
			transform(ii,j);
			deviation(&pos,&dev,&ok,&sum1);
			if (!ok)
				break;
			if (dev<(min_dev*2)) {
				flags[ii+1] = NULL_BRKPT;
				flags[ii] = OK;
				i = ii;
			}
			else
				break;
		}

		for (jj=j+1; jj<finish_in-1; jj++) {
			if (flags[jj+1] == NULL_BRKPT)
				break;
  			transform(i,jj);
			deviation(&pos,&dev,&ok,&sum1);
			if (!ok)
				break;
			if (dev<(min_dev*2)) {
				flags[jj-1] = NULL_BRKPT;
				flags[jj] = OK;
				j = jj;
			}
			else
				break;
		}

		i = max (j+1, iii);
	}
}


bool _EstGround (point3fi *pts, unsigned char *segflg, double *WX, double *WY, double *WZ, int &num, double Equation[], double &Error)
{
	bool		ret=true;

	point3fi	*p1, *p2;
	double		wid, minz, maxz, rng;
	int i=-1, ii, iii;
	while (++i < (SCANDATASIZE-1) && num<SCANDATASIZE) {

		if ((segflg[i]==NODEFLG)&&(segflg[i+1]==LINEFLG)) {

			p1 = p2 = &pts[i];
			for (ii=i+1; ii<SCANDATASIZE; ii++) {
				if (segflg[ii]!=LINEFLG)
					break;
				if (!pts[ii].i)
					continue;
				if (!p1->i)
					p1 = p2 = &pts[ii];
				else
					p2 = &pts[ii];
			}
			
			if (!p1->i || !p2->i) {
				i = ii-1;
				continue;
			}

			maxz = max(p1->z,p2->z);
			minz = min(p1->z,p2->z);

			wid = sqrt (sqr(p1->x-p2->x)+sqr(p1->y-p2->y));
			rng = min(p2r(p1),p2r(p2));
			if ((wid>10.0 && maxz<-1.0) || maxz<-2.0) {// || (wid>5.0 && maxz<0.0 && (maxz-(-2.0))/rng<0.015)) {
				for (iii=i; iii<ii; iii++) {
					WX[num] = pts[iii].x;
					WY[num] = pts[iii].y;
					WZ[num] = pts[iii].z;
					num ++;
					if (num>=SCANDATASIZE)
						break;
				}
			}
			i = ii-1;
		}
	}
	if (num<SCANDATASIZE/2)
		ret = false;
	else {
		Calculate_Plane(num,WX,WY,WZ,0,Equation);
		Calculate_Residuals(WX,WY,WZ,Equation,&Error,num);
		ret = true;
	}
	return ret;
}

bool EstimateGround (double Equation[], double &Error)
{
	double *WX = new double [SCANDATASIZE];
	double *WY = new double [SCANDATASIZE];
	double *WZ = new double [SCANDATASIZE];
	int		num=0;

	int sno=rm.len*0.7;
	bool ret=false;
	for (int y=sno; y>=0; y--) {
		SegmentScanLine (&rm.pts[y*rm.wid], &rm.segflg[y*rm.wid]);
		if (ret=_EstGround (&rm.pts[y*rm.wid], &rm.segflg[y*rm.wid], WX, WY, WZ, num, Equation, Error))
			break;
	}
	delete []WX;
	delete []WY;
	delete []WZ;
	return ret;
}


void CorrectPoints (double Equation[])
{
	double	ax, ay, cx, d;

	ax = asin (-Equation[1]);
	cx = cos (ax);
	ay = atan2 (Equation[0]/cx, Equation[2]/cx);
	
	char str[100];
	sprintf (str, "%.6f\t%.6f\n", ax, ay);
//	OutputLog ("rowandpitch.log", str);

	MATRIX		rot;
	createRotMatrix_XYZ(rot, -ax, -ay , 0.0) ; 
	d = Equation[3];

	for (int y=0; y<rm.len; y++) {
		for (int x=0; x<rm.wid; x++) {
			if (!rm.pts[y*rm.wid+x].i)
				continue;
			rotatePoint3fi(rm.pts[y*rm.wid+x], rot);
			rm.pts[y*rm.wid+x].z += d;
		}
	}
}

void LabelBackground ()
{
	double _maxgz = 0.1;
	double _minhz = 4.0;

	for (int y=0; y<rm.len; y++) {
		for (int x=0; x<rm.wid; x++) {
			if (!rm.pts[y*rm.wid+x].i)
				continue;

			double	rng = p2r(&rm.pts[y*rm.wid+x]);
			if (rm.pts[y*rm.wid+x].z<max(_maxgz,0.012*rng)) 
				rm.segflg[y*rm.wid+x] = ROADFLG;
			else
			if (rm.pts[y*rm.wid+x].z>_minhz)
				rm.segflg[y*rm.wid+x] = HIGHFLG;
		}
	}
}

void SmoothingData ()
{
	int maxcnt = 5;
	point3fi	*p1, *p2;

	for (int y=0; y<rm.len; y++) {
		for (int x=1; x<(rm.wid-1); x++) {
			if (rm.pts[y*rm.wid+(x-1)].i && !rm.pts[y*rm.wid+x].i) {

				int xx;
				for (xx=x+1; xx<rm.wid; xx++) {
					if (rm.pts[y*rm.wid+xx].i)
						break;
				}
				if (xx>=rm.wid)
					continue;
				int cnt = xx-x+1;
				if (cnt>maxcnt) {
					x = xx;
					continue;
				}
				point3fi *p1 = &rm.pts[y*rm.wid+(x-1)];
				point3fi *p2 = &rm.pts[y*rm.wid+xx];
				double dis = ppDistance3fi (p1, p2);
				double rng = max(p2r(p1),p2r(p2));
				double maxdis = min(MAXSMOOTHERR, max (BASEERROR*2, HORIERRFACTOR*cnt*rng));
				if (dis<maxdis) {
					for (int xxx=x; xxx<xx; xxx++) {
						point3fi *p = &rm.pts[y*rm.wid+xxx];
						p->x = (p2->x-p1->x)/cnt*(xxx-x+1)+p1->x;
						p->y = (p2->y-p1->y)/cnt*(xxx-x+1)+p1->y;
						p->z = (p2->z-p1->z)/cnt*(xxx-x+1)+p1->z;
						p->i = 1;
					}
				}
				x = xx;
			}
		}
	}
}

