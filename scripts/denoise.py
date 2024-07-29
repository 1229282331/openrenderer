import cv2
import random
import numpy as np
img = cv2.imread('../result.png')

# # 产生高斯随机数
# noise = np.random.normal(0,50,size=img.size).reshape(img.shape[0],img.shape[1],img.shape[2])
# # 加上噪声
# img = img + noise
# img = np.clip(img,0,255)
# img = img/255   

# 算术平均滤波
img1 = np.transpose(img,(2,0,1))  #转换成[channel,H,W]形式
m = 3   #定义滤波核大小
n = 3
rec_img = np.zeros((img1.shape[0],img1.shape[1]-m+1,img1.shape[2]-n+1))
for channel in range(rec_img.shape[0]):
    for i in range(rec_img[channel].shape[0]):
        for j in range(rec_img[channel].shape[1]):
            rec_img[channel][i,j] = img1[channel][i:i+m,j:j+n].sum()/(m*n)
rec_img = np.transpose(rec_img,(1,2,0))

cv2.imshow('average',rec_img)
cv2.waitKey(0)
