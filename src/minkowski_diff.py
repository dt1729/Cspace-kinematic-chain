import numpy as np
import math
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


class point:
    x = 0.0
    y = 0.0
    def __init__(self,x_coord,y_coord) -> None:
        self.x = float(x_coord)
        self.y = float(y_coord)

def part_b():
    partitions = 12
    obstacle = [point(0.0,0.0),point(1.0,2.0),point(0.0,2.0)]
    robot = [point(0.0,0.0),point(1.0,2.0),point(0.0,2.0)]
    rrobot = rotatePoints(robot,partitions)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    tt = [i for i in np.arange(0,2*math.pi,2*math.pi/partitions)]
    count = 0
    for r in rrobot:
        Cspace_obs = minkowski_diff(obstacle,r)
        Cspace_x, Cspace_y = Point2list(Cspace_obs)
        ans = []
        for p in Cspace_obs:
            ans.append([p.x,p.y])
        ans.append([Cspace_obs[0].x, Cspace_obs[0].y])
        ans = np.array(ans)
        hull = ConvexHull(ans)
        for simplex in hull.simplices:
            ax.plot(ans[simplex, 0], ans[simplex, 1],tt[count], 'r-')
        z_val_pt = [tt[count] for _ in range(len(Cspace_x))]
        ax.plot3D(Cspace_x, Cspace_y,z_val_pt, '*b', lw=2)
        count+=1
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('theta')
    plt.title("3D Cspace obstacles of two triangles one with rotation")
    plt.show()

def part_a():
    obstacle = [point(0.0,0.0),point(1.0,2.0),point(0.0,2.0)]
    robot = [point(2.0,2.0),point(3.0,4.0),point(2.0,4.0)]
    Cspace_obs = minkowski_diff(obstacle,robot)
    Cspace_x, Cspace_y = Point2list(Cspace_obs)
    ans = []
    for p in Cspace_obs:
        ans.append([p.x,p.y])
    ans.append([Cspace_obs[0].x, Cspace_obs[0].y])
    ans = np.array(ans)
    hull = ConvexHull(ans)
    for simplex in hull.simplices:
        plt.plot(ans[simplex, 0], ans[simplex, 1], 'r-')
    plt.plot(Cspace_x, Cspace_y,"*")
    plt.plot(ans[hull.vertices[0],0], ans[hull.vertices[0],1], 'ro')
    plt.title("2D Cspace obstacles of two triangles")
    plt.show()

def minkowski_diff(a,b):
    b = move(b,a)
    minus_b = []
    for p in b:
        minus_b.append(point(-p.x,-p.y))
    diff_vec = []
    for i in a:
        for j in minus_b:
            temp = point(0.0,0.0)
            diff_vec.append(point(i.x + j.x, i.y + j.y))
    return diff_vec


def move(a,b):
    dt = -0.0001
    error = 999999
    while(abs(error) >= 0.0001):
        angle = math.atan2(b[0].y - a[0].y, b[0].x - a[0].x)
        for i in range(len(a)):
            a[i].x -= dt*math.cos(angle)
            a[i].y -= dt*math.sin(angle)
        error = ((a[0].x - b[0].x)**2 + (a[0].y - b[0].y)**2)**0.5

    return a

def Point2list(a):
    ans_x, ans_y = [], []
    for p in a:
        ans_x.append(p.x)
        ans_y.append(p.y)
    
    return ans_x, ans_y


def rotatePoints(robot, partitions):
    ans = []
    for i in range(1,partitions):
        temp = []
        for P in robot:
            tempP = point(0.0,0.0)
            tempP.x = P.x*math.cos(2*math.pi/partitions * i) - P.y*math.sin(2*math.pi/partitions * i)
            tempP.y = P.x*math.sin(2*math.pi/partitions * i) + P.y*math.cos(2*math.pi/partitions * i)
            temp.append(tempP)
        ans.append(temp)
    return ans


part_a()
part_b()