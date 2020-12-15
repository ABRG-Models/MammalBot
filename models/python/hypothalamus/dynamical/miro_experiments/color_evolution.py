import cv2
import numpy as np 

t_f = 0.1
im = cv2.imread("balls.png")
m,n,r = im.shape
h = 0.01
n_it = int(t_f/h)
print n_it
im_r = cv2.normalize(im, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
a = 2.0/3.0
b = 4.0/3.0
c = 1.0
d = 1.0
p = 0.3

for k in range(n_it):
    print "iteration: ", k
    for i in range(m):
        for j in range(n):
            x = im_r[i,j,0]
            y = im_r[i,j,1]
            z = im_r[i,j,2]
            
            im_r[i,j,0] += h*(a*x - b*x*y)
            im_r[i,j,1] += h*(c*x*y - d*y)
            im_r[i,j,2] += -h*p*z 

print im_r

cv2.imshow( "balls", im_r )
cv2.waitKey(0)