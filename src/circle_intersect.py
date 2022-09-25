import math
import numpy as np
import matplotlib.pyplot as plt


xs = [2.2,1.2] 
ys = [15.9,3.0]
xc,yc = 2.0,2.0
xintr, yintr = 0.0,0.0
xsmax, ysmax = 0.0,0.0
xsmin, ysmin = 0.0,0.0
d = 2.0

if(abs(xs[0] - xs[1]) != 0.0):
    m = (ys[1] - ys[0])/(xs[1] - xs[0])
    c = ys[0] - (m)*xs[0]
else:
    m = 0
    c = ys[0] - (m)*xs[0]
    
print(m,c)
if(abs(xs[1] - xs[0]) != 0.0):
    x1 =(-(m*(c-yc) - xc) + (math.sqrt((m*(c-yc) - xc)**2 - (1+m**2)*(xc**2 + (c-yc)**2 -d**2))))/(1+m**2)
    x2 =(-(m*(c-yc) - xc) - (math.sqrt((m*(c-yc) - xc)**2 - (1+m**2)*(xc**2 + (c-yc)**2 -d**2))))/(1+m**2)

    y1 = m*x1 + c
    y2 = m*x2 + c
    print("here")
else:
    x1 = xs[0]
    x2 = xs[0]

    y1 = math.sqrt(d**2 - (x1-xc)**2) + yc
    y2 = -math.sqrt(d**2 - (x2-xc)**2) + yc
    

if(xs[0] > xs[1]):
    xsmin = xs[1]
    xsmax = xs[0]
else:
    xsmin = xs[0]
    xsmax = xs[1]

if(ys[0] > ys[1]):
    ysmin = ys[1]
    ysmax = ys[0]
else:
    ysmin = ys[0]
    ysmax = ys[1]

if((xsmin <= x1 and x1 <= xsmax) and (ysmin <= y1 and y1 <= ysmax)):
    xintr = x1
    yintr = y1
else:
    xintr = x2
    yintr = y2

print(x1, y1, math.sqrt((xc-x1)**2 + (yc-y1)**2))
print(x2, y2, math.sqrt((xc-x2)**2 + (yc-y2)**2))
print(xintr, yintr)
