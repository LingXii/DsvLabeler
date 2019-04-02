#include "DsvLabelMv.h"
#include "pclmethod.h"
#include <fstream>
#include <QString>
#include <string>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#define RG 1
#define LCCP 2
int MODE = LCCP;

TRANSINFO	calibInfo;

std::ifstream dfp;

//int		dsbytesiz = sizeof (point3d)*2 + sizeof (ONEVDNDATA);
int		dsbytesiz = sizeof (ONEDSVDATA);
int		dFrmNum=0;
int		dFrmNo=0;
string img_path = "images/";

bool	videooutput = true;
CvVideoWriter* riFrameWriter = NULL;
CvVideoWriter* seFrameWriter = NULL;
CvVideoWriter* clFrameWriter = NULL;
CvVideoWriter* gtFrameWriter = NULL;

FILE	*seglogfp=NULL;

RMAP	rm;

ONEDSVFRAME	*onefrm;

#define	COLORNUM		17
unsigned char	LEGENDCOLORS[COLORNUM][3] =
			{
				{128,255,255},{0,0,255},{255,0,0},{0,255,0},
                {128,128,255},{255,255,0},{255,164,164},{255,0,255},{0,255,255},
				{128,0,128},{255,128,0},{255,128,255},
				{164,255,255},
                {255,64,64},{0,128,128},{128,128,0},{0,0,0}

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

float to_rgb(int x) // transfer label x to float rgb format
{
    int val = x%COLORNUM;
    int r = LEGENDCOLORS[val][2];
    int g = LEGENDCOLORS[val][1];
    int b = LEGENDCOLORS[val][0];
    int i_rgb = r<<16 | g<<8 | b;
    int* p = &i_rgb;
    float f_rgb = *((float*)p);
    return f_rgb;
}

void transformToPclFormat()
{
    int x, y;
    double	rng;
    point3fi *p;
    int i_time = onefrm->dsv[0].millisec;
    string s_time = to_string(i_time);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<pcl::PointXYZRGBL> ccloud;
    cloud.width    = LINES_PER_BLK*BKNUM_PER_FRM;
    cloud.height   = PNTS_PER_LINE;
    cloud.is_dense = false;
    ccloud.width    = LINES_PER_BLK*BKNUM_PER_FRM;
    ccloud.height   = PNTS_PER_LINE;
    ccloud.is_dense = false;
    int num = 0;
    cloud.points.resize (cloud.width * cloud.height);
    ccloud.points.resize (cloud.width * cloud.height);
    for (y=0; y<rm.len; y++) {
        for (x=0; x<rm.wid; x++) {
            cloud.points[num].x = rm.pts[y*rm.wid+x].x;
            cloud.points[num].y = rm.pts[y*rm.wid+x].y;
            cloud.points[num].z = rm.pts[y*rm.wid+x].z;
            cloud.points[num].intensity =  rm.pts[y*rm.wid+x].i;
            ccloud.points[num].x = rm.pts[y*rm.wid+x].x;
            ccloud.points[num].y = rm.pts[y*rm.wid+x].y;
            ccloud.points[num].z = rm.pts[y*rm.wid+x].z;
            ccloud.points[num].rgb = to_rgb(rm.gt[y*rm.wid+x]);
            ccloud.points[num].label = rm.gt[y*rm.wid+x];
            num += 1;
        }
    }
    string name = "pcd/" + s_time + ".pcd";
    pcl::io::savePCDFileASCII(name,cloud);
    string cname = "pcd/" + s_time + "_c.pcd";
    pcl::io::savePCDFileASCII(cname,ccloud);
}

void ProcessOneFrame ()
{
	int x, y;
	double	rng;
	point3fi *p;
    int i_time = onefrm->dsv[0].millisec;
    string s_time = to_string(i_time);
    string gt_end = "_gt.png";
    cv::Mat gt_img;
    gt_img = cv::imread(img_path + s_time + gt_end);

	//����һ֡vel32���ݡ�����������ϵ����

	//���ӻ�λͼ
	cvZero(rm.rMap);	//����ͼ��
	cvZero(rm.lMap);	//�ָ�ͼ��
	cvZero(rm.dMap);	//���ݹ���ͼ��
	cvZero(rm.sMap);	//����ͼ��
    cvZero(rm.gtMap);	//ground truthͼ��
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
                rng=sqrt(SQR(p->x)+SQR(p->y)+SQR(p->z));
				x=i*LINES_PER_BLK+j;
				y=k;
				rm.pts[y*rm.wid+x] = onefrm->dsv[i].points[j*PNTS_PER_LINE+k];
				rm.rMap->imageData[y*rm.wid+x] = min(255,int(rng*10));                
                lab = onefrm->dsv[i].lab[j*PNTS_PER_LINE+k];
                rm.gt[y*rm.wid+x] = gt_img.at<uchar>(y,x/2,0);
                if (lab > 0)
                    idx_set.insert(lab);
			}
		}
	}

    if(MODE == RG)
    {
        //��ÿһ�������ж�����Ч����㣨cnt<5��Լˮƽ1��)�����ڲ岹�룬������Щ��Ч�㴦�ᱻ��Ϊ�Ǳ߽��
        SmoothingData ();
        transformToPclFormat();

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
    }
    else if(MODE == LCCP)
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_pc = LCCP_seg(s_time);

        SmoothingData ();
        static double	Equation[4]={0,0,1,0};
        double		Error;
        EstimateGround (Equation, Error);
        CorrectPoints (Equation);
        //��ע�����
        LabelBackground ();
        ContourExtraction();

        for (y=0; y<rm.len; y++) {
            for (x=0; x<rm.wid; x++) {
                int k = y*rm.wid+x;
                if(rm.regionID[k] != NONVALID && rm.regionID[k] != GROUND){
                    rm.regionID[k] = labeled_pc->points[k].label;
                    rm.regnum = max(rm.regnum,rm.regionID[k]+1);
                }
            }
        }
        rm.segbuf = new SEGBUF[rm.regnum];
        memset (rm.segbuf, 0, sizeof (SEGBUF)*rm.regnum);
        EstimateSeg ();
        ClassiSeg ();
    }

	//���ָ������������ӻ�λͼ��ɫ
	unsigned short val;
	for (y=0; y<rm.len; y++) {
		for (x=0; x<rm.wid; x++) {
			if (rm.regionID[y*rm.wid+x]==EDGEPT) {
                rm.lMap->imageData[(y*rm.wid+x)*3+2]	= 128;
                rm.lMap->imageData[(y*rm.wid+x)*3+1]	= 128;
                rm.lMap->imageData[(y*rm.wid+x)*3+0]	= 128;
                rm.sMap->imageData[(y*rm.wid+x)*3+2]	= 0;
                rm.sMap->imageData[(y*rm.wid+x)*3+1]	= 255;
                rm.sMap->imageData[(y*rm.wid+x)*3+0]	= 0;
			}
			else if (rm.regionID[y*rm.wid+x]==GROUND)
			{
				rm.lMap->imageData[(y*rm.wid+x)*3+2]	= 0;
				rm.lMap->imageData[(y*rm.wid+x)*3+1]	= 0;
				rm.lMap->imageData[(y*rm.wid+x)*3+0]	= 128;
                rm.sMap->imageData[(y*rm.wid+x)*3+2]	= 164;
                rm.sMap->imageData[(y*rm.wid+x)*3+1]	= 164;
                rm.sMap->imageData[(y*rm.wid+x)*3+0]	= 255;
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
            val = rm.gt[y*rm.wid+x]%COLORNUM;
            if(val==0) val=16;
            rm.gtMap->imageData[(y*rm.wid+x)*3+2]	= LEGENDCOLORS[val][2];
            rm.gtMap->imageData[(y*rm.wid+x)*3+1]	= LEGENDCOLORS[val][1];
            rm.gtMap->imageData[(y*rm.wid+x)*3+0]	= LEGENDCOLORS[val][0];
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
    rm->gt = new int[rm->wid*rm->len];
	rm->pts = new point3fi[rm->wid*rm->len];
    rm->segflg = new unsigned char[rm->wid*rm->len];
	rm->regionID = new int[rm->wid*rm->len];
	rm->segbuf = NULL;
	rm->rMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 1);
	rm->lMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 3);
	rm->dMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 3);
	rm->sMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 3);
    rm->gtMap = cvCreateImage(cvSize(rm->wid,rm->len), IPL_DEPTH_8U, 3);
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
        gtFrameWriter = cvCreateVideoWriter("gt.avi",CV_FOURCC('M','J','P','G'), 15, cvSize (rm.wid/2, rm.len*4.5));
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
        int i_time = onefrm->dsv[0].millisec;
        string s_time = to_string(i_time);

		if (rm.regnum>=10000) {
			printf("Warning: regionnum (%d). Can not exceed 10000\n",rm.regnum);
		}

		//���������ⲿ�ļ�
        //Write2LabelFile ();

		//���ӻ�
        if (videooutput)
        {
            cvResize (rm.rMap, out);
            cvWriteFrame(riFrameWriter, out);
            char str[10];
            sprintf (str, "Fno%d", dFrmNo);
            cvPutText(out, str,cvPoint(50,50),&font,CV_RGB(0,0,255));
            cvShowImage("range image",out);
            cvResize (rm.lMap, col);
            cvWriteFrame(seFrameWriter, col);
    //		cvShowImage("segmentation",col);
            cvResize (rm.dMap, col);
    //		cvShowImage("data association",col);
            cvResize (rm.gtMap, col);
            cvShowImage("ground truth",col);
            cvWriteFrame(gtFrameWriter, col);
            cvResize (rm.sMap, col);
            cvShowImage("classification",col);
            cvWriteFrame(clFrameWriter, col);

            cv::Mat SMAP(rm.sMap);
            cv::Size dsize = cv::Size(1080, 32);
            cv::resize(SMAP,SMAP,dsize,0,0,cv::INTER_NEAREST);
            cv::imwrite("images/"+s_time+"_rg.png",SMAP);
            cv::Mat GTMAP(rm.gtMap);
            cv::resize(GTMAP,GTMAP,dsize,0,0,cv::INTER_NEAREST);
            cv::imwrite("images/"+s_time+"_merge.png",GTMAP);
        }

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
    if (gtFrameWriter)	cvReleaseVideoWriter(&gtFrameWriter);

	ReleaseRmap (&rm);
	ReleaseDmap ();
	cvReleaseImage(&out);
	cvReleaseImage(&col);
	delete []onefrm;
}

int main (int argc, char *argv[])
{

    std::string dsvlfilename = "2-2-bg.dsvl";
    //std::string dsvlfilename = "/home/pku-m/SemanticMap/Data/FineAnnotation/2-1/2-1.dsvl";
    //std::string dsvlfilename = "/media/pku-m/OrangePassport4T1/SemanticDataCollect/20170410_campus/origin/campus2/3D/2.dsv";

    std::string calibfilename = "calib32.txt";

    std::string outseglogfilename = "2-2-classified.log";

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
