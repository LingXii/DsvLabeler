#include "DsvLabelMv.h"
#include <fstream>
#include <QString>

#include <unordered_set>

TRANSINFO	calibInfo;

std::ifstream dfp;

//int		dsbytesiz = sizeof (point3d)*2 + sizeof (ONEVDNDATA);
int		dsbytesiz = sizeof (ONEDSVDATA);
int		dFrmNum=0;
int		dFrmNo=0;

bool	videooutput = false;
CvVideoWriter* riFrameWriter = NULL;
CvVideoWriter* seFrameWriter = NULL;
CvVideoWriter* clFrameWriter = NULL;

FILE	*seglogfp=NULL;

RMAP	rm;

ONEDSVFRAME	*onefrm;

#define	COLORNUM		16
unsigned char	LEGENDCOLORS[COLORNUM][3] =
			{
				{128,255,255},{0,0,255},{255,0,0},{0,255,0},
				{128,128,255},{255,255,0},{0,128,255},{255,0,255},{0,255,255},
				{128,0,128},{255,128,0},{255,128,255},
				{164,255,255},
				{255,64,64},{0,128,128},{128,128,0},

			};

bool LoadCalibFile (const char *szFile)
{
	char			i_line[200];
    FILE			*fp;
	MATRIX			rt;

	fp = fopen (szFile, "r");
	if (!fp)
		return false;

	rMatrixInit (calibInfo.rot);

	while (1) {
		if (fgets (i_line, 80, fp) == NULL)
			break;

        if (strncasecmp (i_line, "rot", 3) == 0) {
			strtok (i_line, " ,\t\n");
			calibInfo.ang.x = atof (strtok (NULL, " ,\t\n"))*topi;
			calibInfo.ang.y = atof (strtok (NULL, " ,\t\n"))*topi;
			calibInfo.ang.z = atof (strtok (NULL, " ,\t\n"))*topi;
			createRotMatrix_ZYX (rt, calibInfo.ang.x, calibInfo.ang.y, calibInfo.ang.z);
			rMatrixmulti (calibInfo.rot, rt);
			continue;
		}

        if (strncasecmp (i_line, "shv", 3) == 0) {
			strtok (i_line, " ,\t\n");
			calibInfo.shv.x = atof (strtok (NULL, " ,\t\n"));
			calibInfo.shv.y = atof (strtok (NULL, " ,\t\n"));
			calibInfo.shv.z = atof (strtok (NULL, " ,\t\n"));
		}
	}
	fclose (fp);

	return true;
}

void ProcessOneFrame ()
{
	int x, y;
	double	rng;
	point3fi *p;

	//����һ֡vel32���ݡ�����������ϵ����

	//���ӻ�λͼ
	cvZero(rm.rMap);	//����ͼ��
	cvZero(rm.lMap);	//�ָ�ͼ��
	cvZero(rm.dMap);	//���ݹ���ͼ��
	cvZero(rm.sMap);	//����ͼ��
	memset (rm.pts, 0, sizeof (point3fi)*rm.wid*rm.len);	//����ͼ���Ӧ�ļ��������
    memset (rm.segflg, 0, sizeof (unsigned char)*rm.wid*rm.len);		//����ͼ���Ӧ�ķ����ǩ����
	memset (rm.regionID, 0, sizeof (int)*rm.wid*rm.len);	//����ͼ���Ӧ�ķָ��ǩ����
	rm.regnum = 0;
										//�ָ��ǩ��Ŀ
	rm.ang = onefrm->ang;
	rm.shv = onefrm->shv;

	//���ɾ���ͼ����Ӧ�����ݡ���180��12����32
    int lab;

    std::unordered_set<int> idx_set;

	for (int i=0; i<BKNUM_PER_FRM; i++) {
		for (int j=0; j<LINES_PER_BLK; j++) {
			for (int k=0; k<PNTS_PER_LINE; k++) {
				p = &onefrm->dsv[i].points[j*PNTS_PER_LINE+k];
				if (!p->i)
					continue;
				rng=sqrt(sqr(p->x)+sqr(p->y)+sqr(p->z));
				x=i*LINES_PER_BLK+j;
				y=k;
				rm.pts[y*rm.wid+x] = onefrm->dsv[i].points[j*PNTS_PER_LINE+k];
				rm.rMap->imageData[y*rm.wid+x] = min(255,int(rng*10));
                lab = onefrm->dsv[i].lab[j*PNTS_PER_LINE+k];
                if (lab > 0)
                    idx_set.insert(lab);
			}
		}
	}

	//��ÿһ�������ж�����Ч����㣨cnt<5��Լˮƽ1��)�����ڲ岹�룬������Щ��Ч�㴦�ᱻ��Ϊ�Ǳ߽��
	SmoothingData ();

	//���ݴ��µĴ������߶ȣ��ҵ������ĵ���㣬��ϵ�ƽ�棬��ƽ�汣�浽Equation��
	static double	Equation[4]={0,0,1,0};
	double		Error;
	EstimateGround (Equation, Error);

	//����Equation����������㣬ʹ�õ���ˮƽ������߶�Ϊ0��ʹ�ø߶���Ϣ�����ڷ���
	CorrectPoints (Equation);

	//��ע�����
	LabelBackground ();

	//�ָ�
	//��һ������ע�߽��ContourExtraction();
	//�ڶ���������������ʽ��ע�����ڵ�RegionGrow()
	//�����������߽��ϲ�����������EdgeGrow ()
	ContourSegger ();

	//Ϊÿ����������һ��segbuf�����ڷ��ࡢĿǰ����ȡ����������
	EstimateSeg ();

    DataAssociation ();

	//���ࡢĿǰ�����������Ŀ�͸߽��з���
	ClassiSeg ();

	//���ָ������������ӻ�λͼ��ɫ
	unsigned short val;
	for (y=0; y<rm.len; y++) {
		for (x=0; x<rm.wid; x++) {
			if (rm.regionID[y*rm.wid+x]==EDGEPT) {
				rm.lMap->imageData[(y*rm.wid+x)*3+2]	= 128;
				rm.lMap->imageData[(y*rm.wid+x)*3+1]	= 128;
				rm.lMap->imageData[(y*rm.wid+x)*3+0]	= 128;
			}
			else if (rm.regionID[y*rm.wid+x]==GROUND)
			{
				rm.lMap->imageData[(y*rm.wid+x)*3+2]	= 0;
				rm.lMap->imageData[(y*rm.wid+x)*3+1]	= 0;
				rm.lMap->imageData[(y*rm.wid+x)*3+0]	= 128;
			}
			else if (rm.regionID[y*rm.wid+x]==NONVALID)
			{
				rm.lMap->imageData[(y*rm.wid+x)*3+2]	= 255;
                rm.lMap->imageData[(y*rm.wid+x)*3+1]	= 255;
				rm.lMap->imageData[(y*rm.wid+x)*3+0]	= 255;
			}
			else if (rm.regionID[y*rm.wid+x]==UNKNOWN)
			{
				rm.lMap->imageData[(y*rm.wid+x)*3+2]	= 64;
				rm.lMap->imageData[(y*rm.wid+x)*3+1]	= 64;
				rm.lMap->imageData[(y*rm.wid+x)*3+0]	= 64;
			}
			else if (rm.regionID[y*rm.wid+x]==BACKGROUND)
			{
				val = rm.pts[y*rm.wid+x].i;
				rm.lMap->imageData[(y*rm.wid+x)*3+2]	= val;
				rm.lMap->imageData[(y*rm.wid+x)*3+1]	= val;
				rm.lMap->imageData[(y*rm.wid+x)*3+0]	= val;
			}
			else {
				val = (rm.regionID[y*rm.wid+x])%COLORNUM;
				rm.lMap->imageData[(y*rm.wid+x)*3+2]	= LEGENDCOLORS[val][2];
				rm.lMap->imageData[(y*rm.wid+x)*3+1]	= LEGENDCOLORS[val][1];
				rm.lMap->imageData[(y*rm.wid+x)*3+0]	= LEGENDCOLORS[val][0];

				if (rm.segbuf[rm.regionID[y*rm.wid+x]].prid) {
					val = rm.segbuf[rm.regionID[y*rm.wid+x]].prid%COLORNUM;
					rm.dMap->imageData[(y*rm.wid+x)*3+2]	= LEGENDCOLORS[val][2];
					rm.dMap->imageData[(y*rm.wid+x)*3+1]	= LEGENDCOLORS[val][1];
					rm.dMap->imageData[(y*rm.wid+x)*3+0]	= LEGENDCOLORS[val][0];
				}

				val = rm.segbuf[rm.regionID[y*rm.wid+x]].lab;
				rm.sMap->imageData[(y*rm.wid+x)*3+2]	= LEGENDCOLORS[val][2];
				rm.sMap->imageData[(y*rm.wid+x)*3+1]	= LEGENDCOLORS[val][1];
				rm.sMap->imageData[(y*rm.wid+x)*3+0]	= LEGENDCOLORS[val][0];
			}
		}
	}
}

void OutputLog (char *filename, char *str)
{
	FILE	*fp;

	fp = fopen (filename, "a");
	if (!fp)
		return;

	fprintf (fp, "%s", str);

	fclose (fp);
}

//��ȡһ֡vel32���ݣ�һ֡Ϊ180��12��32������㣩���浽onefrm->dsv��δ������ת��
bool ReadOneDsvFrame ()
{
	int		i;

	onefrm->ang.x = onefrm->ang.y = onefrm->ang.z = 0;
	onefrm->shv.x = onefrm->shv.y = onefrm->shv.z = 0;

	for (i=0; i<BKNUM_PER_FRM; i++) {
        dfp.read((char *)&onefrm->dsv[i], dsbytesiz);

        if (dfp.gcount() != dsbytesiz)
            break;

		onefrm->ang.x += onefrm->dsv[i].ang.x;
		onefrm->ang.y += onefrm->dsv[i].ang.y;
		onefrm->ang.z += onefrm->dsv[i].ang.z;
		onefrm->shv.x += onefrm->dsv[i].shv.x;
		onefrm->shv.y += onefrm->dsv[i].shv.y;
		onefrm->shv.z += onefrm->dsv[i].shv.z;
	}

	//ÿһ֡��һ��ƽ����λ�ˣ��������ݹ���
	onefrm->ang.x /= BKNUM_PER_FRM;
	onefrm->ang.y /= BKNUM_PER_FRM;
	onefrm->ang.z /= BKNUM_PER_FRM;
	onefrm->shv.x /= BKNUM_PER_FRM;
	onefrm->shv.y /= BKNUM_PER_FRM;
	onefrm->shv.z /= BKNUM_PER_FRM;

	if (i<BKNUM_PER_FRM)
        return 0;
	else
        return 1;
}

void InitRmap (RMAP *rm)
{
	rm->wid = LINES_PER_BLK*BKNUM_PER_FRM;
	rm->len = PNTS_PER_LINE;
	rm->pts = new point3fi[rm->wid*rm->len];
    rm->segflg = new unsigned char[rm->wid*rm->len];
	rm->regionID = new int[rm->wid*rm->len];
	rm->segbuf = NULL;
	rm->rMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 1);
	rm->lMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 3);
	rm->dMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 3);
	rm->sMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 3);
}

void ReleaseRmap (RMAP *rm)
{
	delete []rm->pts;
	delete []rm->segflg;
	delete []rm->regionID;
	if (rm->segbuf)
		delete []rm->segbuf;
	cvReleaseImage(&rm->rMap);
	cvReleaseImage(&rm->lMap);
	cvReleaseImage(&rm->sMap);
	cvReleaseImage(&rm->dMap);
}


void JumpTo (int fno)
{
    uint64 li;
    //jump frame
    li = (dsbytesiz)*BKNUM_PER_FRM*fno;
    dfp.seekg(li, std::ios::beg);
    dFrmNum = fno;
}

//���������
void DoLabeling()
{
    //move pointer to end
    dfp.seekg(0,ios::end);
    uint64 numOfBytes = dfp.tellg() /180 / dsbytesiz;
    std::cout<<"Total Dsv Frame Num: "<<numOfBytes<<std::endl;

    //���ļ�ָ���ƶ����ļ���ͷ
    dfp.clear();
    dfp.seekg(0,std::ios::beg);

	InitRmap (&rm);
	InitDmap ();
	onefrm= new ONEDSVFRAME[1];

	IplImage * out = cvCreateImage (cvSize (rm.wid/2, rm.len*4.5),IPL_DEPTH_8U,1);
	IplImage * col = cvCreateImage (cvSize (rm.wid/2, rm.len*4.5),IPL_DEPTH_8U,3);

	if (videooutput) {
		riFrameWriter = cvCreateVideoWriter("ri.avi",CV_FOURCC('M', 'J', 'P', 'G'), 15, cvSize (rm.wid/2, rm.len*4.5));
		seFrameWriter = cvCreateVideoWriter("se.avi",CV_FOURCC('M','J','P','G'), 15, cvSize (rm.wid/2, rm.len*4.5));
		clFrameWriter = cvCreateVideoWriter("cl.avi",CV_FOURCC('M','J','P','G'), 15, cvSize (rm.wid/2, rm.len*4.5));
	}

	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX, 1,1, 0, 2);

    int waitkeydelay=1;

//	JumpTo (dFrmNum/2);
	dFrmNo = 0;

	while (ReadOneDsvFrame ())
	{
		if (seglogfp)
			fprintf (seglogfp, "fno=%d,%d\n", dFrmNo, onefrm->dsv[0].millisec);

		if (dFrmNo%100==0)
			printf("%d (%d)\n",dFrmNo,dFrmNum);

        printf("%d\n", onefrm->dsv[0].millisec);
		//ÿһ֡�Ĵ���
		ProcessOneFrame ();

		if (rm.regnum>=10000) {
			printf("Warning: regionnum (%d). Can not exceed 10000\n",rm.regnum);
		}

		//���������ⲿ�ļ�
        //Write2LabelFile ();

		//���ӻ�
		cvResize (rm.rMap, out);
		if (videooutput)
			cvWriteFrame(riFrameWriter, out);
		char str[10];
		sprintf (str, "Fno%d", dFrmNo);
		cvPutText(out, str,cvPoint(50,50),&font,CV_RGB(0,0,255));
		cvShowImage("range image",out);
		cvResize (rm.lMap, col);
		if (videooutput)
			cvWriteFrame(seFrameWriter, col);
		cvShowImage("segmentation",col);
		cvResize (rm.dMap, col);
		cvShowImage("data association",col);

		//if (videooutput)
		//	cvWriteFrame(clFrameWriter, col);
		cvResize (rm.sMap, col);
		cvShowImage("classification",col);
		if (videooutput)
			cvWriteFrame(clFrameWriter, col);

		char WaitKey;
		WaitKey = cvWaitKey(waitkeydelay);
		if (WaitKey==27)
			break;
		if (WaitKey=='z') {
			if (waitkeydelay==1)
				waitkeydelay=0;
			else
				waitkeydelay=1;
		}
		dFrmNo++;
//		if (dFrmNo>dFrmNum/2)
//			break;
	}
	if (riFrameWriter)	cvReleaseVideoWriter(&riFrameWriter);
	if (seFrameWriter)	cvReleaseVideoWriter(&seFrameWriter);
	if (clFrameWriter)	cvReleaseVideoWriter(&clFrameWriter);

	ReleaseRmap (&rm);
	ReleaseDmap ();
	cvReleaseImage(&out);
	cvReleaseImage(&col);
	delete []onefrm;
}

//����������½��зָ�͹���
int main (int argc, char *argv[])
{
    videooutput = 1; //
    std::string dsvlfilename = "/media/pku-m/Data/dsvldata/20170410-2/2-1-mv.dsvl";
    //std::string dsvlfilename = "/home/pku-m/SemanticMap/Data/FineAnnotation/2-1/2-1.dsvl";
    //std::string dsvlfilename = "/media/pku-m/OrangePassport4T1/SemanticDataCollect/20170410_campus/origin/campus2/3D/2.dsv";

    std::string calibfilename = "/home/pku-m/SemanticMapHL/Tools/SImpleClassifier/DsvLabeler/calib32.txt";

    std::string outseglogfilename = "2-1-classified.log";

    //dsv has no label infomation and the onefrm->dsv[i].lab[j*PNTS_PER_LINE+k] is empty
    if (QString(dsvlfilename.c_str()).endsWith("dsv"))
        dsbytesiz = sizeof (point3d)*2 + sizeof (ONEVDNDATA);

    //dsvl has label infomation and the onefrm->dsv[i].lab[j*PNTS_PER_LINE+k] is regionID after dataassociate
    else if (QString(dsvlfilename.c_str()).endsWith("dsvl"))
        dsbytesiz = sizeof (ONEDSVDATA);

    if (!LoadCalibFile (calibfilename.c_str())) {
        fprintf (stderr,"Invalid calibration file : %s.\n", argv[2]);
        getchar ();
        exit (1);
    }

    dfp.open(dsvlfilename.c_str(), std::ios_base::binary);
    if (!dfp.is_open()){
        printf("File open failure : %s\n", dsvlfilename.c_str());
        getchar ();
        exit (1);
    }

    seglogfp = fopen (outseglogfilename.c_str(), "w");

	DoLabeling ();

	fprintf (stderr, "Labeling succeeded.\n");

	fclose (seglogfp);

    dfp.close();

    return 1;
}
