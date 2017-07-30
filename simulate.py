import matplotlib.pyplot as plt
import numpy as np
from numpy import cos, sin
f = open("../robot/data.out", "r")
B, ch, wheel_r = map(float, f.readline().strip().split())
maxv, maxw = map(float, f.readline().strip().split())
diffR = ch * B / 2;

(tick,) = map(float, f.readline().strip().split())
nowx, nowy, nowd, starttime = map(float, f.readline().strip().split())


plt.figure(0)
plt.axis('equal')

cpx = []
cpy = []
cpdir = []
f.readline()
while True:
	l = f.readline()
	if l.strip() == "":
		break
	x, y, dir, t, canStop = map(float, l.strip().split())
	cpx.append(x)
	cpy.append(y)
	cpdir.append(dir)

pathx = []
pathy = []
f.readline()
while True:
	l = f.readline()
	if l.strip() == "":
		break
	x, y, t = map(float, l.strip().split())
	pathx.append(x)
	pathy.append(y)
plt.plot(pathx, pathy, c='r', linewidth = 10, alpha = 0.2)


angular_ws = []
linear_vs = []
rx = [0]
ry = [0]
for line in f.readlines():
	wl, wr = map(float, line.strip().split())
	disl = wl * wheel_r * tick
	disr = wr * wheel_r * tick
	alpha = (disl - disr) / 2 / diffR
	if alpha > 1e-2:
		R = (disl+disr)*diffR/(disl-disr)
		dis = R * alpha
		dx = (1-cos(alpha))*R
		dy = sin(alpha)*R
	else:
		dis = (disl + disr) / 2
		dx = (disl**2 - disr **2) / 8 / diffR
		dy = (disl + disr) / 2
	dx, dy = cos(nowd)*dx+sin(nowd)*dy, -sin(nowd)*dx+cos(nowd)*dy
	nowd+=alpha
	#print nowx, nowy, dx, dy, dis / tick
	#plt.plot([nowx, nowx+dx], [nowy, nowy+dy], c='b')
	
	angular_ws.append(alpha / tick / maxw)
	linear_vs.append(dis / tick / maxv)
	nowx += dx
	nowy += dy
	rx.append(nowx)
	ry.append(nowy)

plt.plot(rx, ry, c='b')

plt.scatter(cpx,cpy,c='black',marker='x')
xmin, xmax, ymin, ymax = plt.axis()
d = max(ymax-ymin, xmax-xmin)
for x, y, dir in zip(cpx, cpy, cpdir):
	if dir != float("inf"):
		dx = sin(dir) * 0.05 * d
		dy = cos(dir) * 0.05 * d
		plt.arrow(x,y,dx,dy, width=0.0003 * d, facecolor='black')

plt.arrow(nowx,nowy,sin(nowd) * 0.1 * d,cos(nowd) * 0.1 * d, width=0.0006 * d, facecolor='blue')


plt.figure(1)
plt.plot([starttime + i * tick for i in range(len(angular_ws))], angular_ws)
plt.title("w")
plt.figure(2)
plt.plot([starttime + i * tick for i in range(len(linear_vs))], linear_vs)
plt.title("v")
plt.show()