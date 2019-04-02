import os
import numpy as np
import cv2 as cv

WORK_PATH = "LCCP/"
H = 32
W = 1080
S = H*W
def judge(label,img):
    mask = np.zeros((H,W,3))
    mask[::] = label
    judge_img = img-mask
    judge_img = np.any(judge_img,axis=2)
    return np.logical_not(judge_img)

UNKNOWN = np.array([0,0,0])
PEOPLE = np.array([0,0,255])
TREE = np.array([0,255,0])
CAR = np.array([255,0,0])
BUILDING = np.array([255,255,0])
SIGN = np.array([128,128,255])
LABEL_SET = [UNKNOWN,PEOPLE,CAR,TREE,SIGN,BUILDING]
LABEL_N = len(LABEL_SET)
IMAGE_PATH = WORK_PATH+"images/"
ctable = np.zeros((LABEL_N,LABEL_N),dtype=np.int32) # (GT,RG)

if(not os.path.exists(WORK_PATH+"ctable.npy")):
    for i,file in enumerate(os.listdir(IMAGE_PATH)):
        if(i%4 != 0): continue
        table = np.zeros((LABEL_N,LABEL_N),dtype=np.int32)
        s_time = file[0:8]
        gt = cv.imread(IMAGE_PATH+s_time+"_merge.png")
        rg = cv.imread(IMAGE_PATH+s_time+"_rg.png")
        judge_gts = []
        judge_rgs = []
        for j,k in enumerate(LABEL_SET):
            judge_gts.append(judge(k,gt))
            judge_rgs.append(judge(k,rg))
        for j in range(LABEL_N):
            for k in range(LABEL_N):
                comp = judge_gts[j] * judge_rgs[k]
                table[j,k] += (np.sum(comp))
        ctable += table
        print(s_time)
    np.save(WORK_PATH+"ctable.npy",ctable)
else:
    print("load ctable file.")
    ctable = np.load(WORK_PATH+"ctable.npy")

print(ctable)
for k in range(1,LABEL_N):
    TP = ctable[k,k]
    FP = np.sum(ctable[1:,k]) - TP # gt unknown is not count
    FN = np.sum(ctable[k,1:]) - TP # rg unknown is not count
    pre = TP/(TP+FP)
    rec = TP/(TP+FN)
    iou = TP/(TP+FP+FN)
    print("label %s: precision %s, recall %s, iou %s"%(k,pre,rec,iou))