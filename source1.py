#%%
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import copy
import math
import time
import cv2
import os
# import scipy.misc
import networkx as nw
from plyfile import *

print('begin')

f=open('./config.txt','r')
lines=f.readlines()
f.close()
conf=[]
for l in lines:
    r=l.split(',')[-1]
    conf.append(r)
# print(lines)

filename=conf[1][:-1]
filename=filename.replace('\\','/')
path=lines[2].split(',')[1][:-1]
# path=conf[3][:-1]
path=path.replace('\\','/')
path=path.strip('\r')

mx=int(conf[5])
densify=bool(eval(conf[7]))
#%%
def str2array(s):
    s=s[1:-2]
    # print(s)
    l=s.split()
    res=[eval(i) for i in l]
    return res

#除去平行且接近的直线
def findparallel(lines,dcri=10):
    directionlist=[]
    parallelpair=[]
    distancelist=[]
    redlines=copy.deepcopy(lines)
    dl=[]
    indexlist=np.array(list(range(len(lines))))
    for i in range(len(lines)):
        for x1, y1, x2, y2 in lines[i]:
            m=((x2-x1)**2+(y2-y1)**2)**0.5
            if m==0:
                m=1
            direction=np.array([(x2-x1)/m,(y2-y1)/m])
            directionlist.append(direction)
    for i in range(len(directionlist)):
        for j in range(i+1,len(directionlist)):
            cosine=np.dot(directionlist[i],directionlist[j])
            if abs(cosine)>0.95:
                x0,y0,x3,y3=lines[i][0][0],lines[i][0][1],lines[i][0][2],lines[i][0][3]
                x1, y1, x2, y2=lines[j][0][0],lines[j][0][1],lines[j][0][2],lines[j][0][3]
                d=abs((y2-y1)*(x0-x1)-(x2-x1)*(y0-y1))/((y2-y1)**2+(x2-x1)**2)**0.5
                if d<dcri:
                    length1s=(y0-y3)**2+(x0-x3)**2
                    length2s=(y1-y2)**2+(x1-x2)**2
                    if length2s<length1s:
                        dl.append(j)
                    else:
                        dl.append(i)
                elif (not i in dl) and (not j in dl):
                    distancelist.append(d)
                    parallelpair.append([i,j])  
    dl2=[]
    for i in range(len(parallelpair)):
        p=parallelpair[i]
        if p[0] in dl or p[1] in dl:
            dl2.append(i)
    distancelist=np.delete(distancelist,dl2,axis = 0) 
    parallelpair=np.delete(parallelpair,dl2,axis = 0) 
    redlines=np.delete(redlines,dl,axis = 0) 
    indexlist=np.delete(indexlist,dl,axis = 0) 
    return directionlist,distancelist,parallelpair,redlines,indexlist


# def findparallel(lines):
#     directionlist=[]
#     parallelpair=[]
#     for i in range(len(lines)):
#         for x1, y1, x2, y2 in lines[i]:
#             m=((x2-x1)**2+(y2-y1)**2)**0.5
#             direction=np.array([(x2-x1)/m,(y2-y1)/m])
#             directionlist.append(direction)
#     for i in range(len(directionlist)):
#         for j in range(i+1,len(directionlist)):
#             cosine=np.dot(directionlist[i],directionlist[j])
#             if abs(cosine)>0.94:
#                 parallelpair.append([i,j])
#     dl=[]
#     distancelist=[]
#     for p in parallelpair:
# #         print(lines[p[1]])
#         x0,y0,x3,y3=lines[p[0]][0][0],lines[p[0]][0][1],lines[p[0]][0][2],lines[p[0]][0][3]
#         x1, y1, x2, y2=lines[p[1]][0][0],lines[p[1]][0][1],lines[p[1]][0][2],lines[p[1]][0][3]
#         d=abs((y2-y1)*(x0-x1)-(x2-x1)*(y0-y1))/((y2-y1)**2+(x2-x1)**2)**0.5
#         if d<20:
#             length1s=(y0-y3)**2+(x0-x3)**2
#             length2s=(y1-y2)**2+(x1-x2)**2
#             if length2s<length1s:
#                 dl.append(p[1])
#             else:
#                 dl.append(p[0])
#         else:
#             distancelist.append(d)
#     lines=np.delete(lines,dl, axis = 0) 
    
#     directionlist=[]
#     newparallelpair=[]
#     for i in range(len(lines)):
#         for x1, y1, x2, y2 in lines[i]:
#             m=((x2-x1)**2+(y2-y1)**2)**0.5
#             direction=np.array([(x2-x1)/m,(y2-y1)/m])
#             directionlist.append(direction)
#     for i in range(len(directionlist)):
#         for j in range(i+1,len(directionlist)):
#             cosine=np.dot(directionlist[i],directionlist[j])
#             if abs(cosine)>0.94:
#                 newparallelpair.append([i,j])
#     return directionlist,distancelist,newparallelpair,lines

def findperpendicular(lines):
    directionlist=[]
    perpendicularpair=[]
    for i in range(len(lines)):
        for x1, y1, x2, y2 in lines[i]:
            m=((x2-x1)**2+(y2-y1)**2)**0.5
            direction=np.array([(x2-x1)/m,(y2-y1)/m])
            directionlist.append(direction)
    for i in range(len(directionlist)):
        for j in range(i+1,len(directionlist)):
            cosine=np.dot(directionlist[i],directionlist[j])
            if abs(cosine)<0.35:
                perpendicularpair.append([i,j])
    return directionlist,perpendicularpair

def cross_point(line1,line2):#计算交点函数
    x1=float(line1[0])
    y1=float(line1[1])
    x2=float(line1[2])
    y2=float(line1[3])
    x3=float(line2[0])
    y3=float(line2[1])
    x4=float(line2[2])
    y4=float(line2[3])
    if x2-x1==0:
        k1=None
        b1=0
    else:
        k1=(y2-y1)*1.0/(x2-x1)
        b1=y1*1.0-x1*k1*1.0
    if x4-x3==0:
        k2=None
        b2=0
    else:
        k2=(y4-y3)*1.0/(x4-x3)
        b2=y3*1.0-x3*k2*1.0
    # print(k1,k2)
    if k1!=k2:
        if k1!=None and k2!=None:
            x=(b2-b1)*1.0/(k1-k2)
            y=k1*x*1.0+b1*1.0
        elif k1==None and k2==None:
            x=np.nan
            y=np.nan
        elif k1!=None and k2==None:
            x=x3
            y=k1*x*1.0+b1*1.0
        elif k1==None and k2!=None:
            x=x1
            y=k2*x*1.0+b2*1.0
    else:
        x=np.nan
        y=np.nan
    return [round(x,5),round(y,5)]

# def check_content(binary,line1,line2):
# #     x1=line1[0]
# #     y1=line1[1]
# #     x2=line1[2]
# #     y2=line1[3]
# #     x3=line2[0]
# #     y3=line2[1]
# #     x4=line2[2]
# #     y4=line2[3]
#     # print(line1)
#     p1=[line1[0][0],line1[0][1]]
#     p2=[line1[0][2],line1[0][3]]
#     p3=[line1[0][0],line1[0][1]]
#     p4=[line1[0][2],line1[0][3]]
#     pl=[p1,p2,p3,p4]
#     midx=sum([pi[0] for pi in pl])/4
#     midy=sum([pi[1] for pi in pl])/4
#     Vertex=[[],[],[],[]]
#     for i in range(4):
#         p=pl[i]
#         if p[0]<midx and p[1]>midy:
#             Vertex[0]=p
#         elif p[0]>midx and p[1]>midy:
#             Vertex[1]=p
#         elif p[0]>midx and p[1]<midy:
#             Vertex[2]=p
#         elif p[0]<midx and p[1]<midy:
#             Vertex[3]=p

#     count=0
#     countw=0
#     b=copy.deepcopy(projectionimg)
#     for i in range(min([pi[0] for pi in pl]),max([pi[0] for pi in pl])):
# #         print(i)
#         for j in range(min([pi[1] for pi in pl]),max([pi[1] for pi in pl])):
#             point=np.array([i,j])
#             c=0
#             for n in range(4):
#                 v1=point-np.array(pl[n])
#                 s1=np.array(pl[(n+1)%4])-np.array(pl[n])
#                 s2=np.array(pl[n])-np.array(pl[(n+3)%4])
# #                 v1=v1[:2]
# #                 s1=s1[:2]
# #                 s2=s2[:2]

#                 if np.dot(np.cross(v1,s1),np.cross(v1,s2))<0:
#                     c+=1
# #             print(c)
#             if c==4:
#                 count+=1
#                 if binary[j][i]==0:
# #                     print(i,j)
#                     cv2.circle(b, [i,j], 1, (0, 0, 255),0)
#                     countw+=1
#     # print(count)
#     if count!=0:
#         return countw/count,b
#     return 0,b

def planefit(p,ps):
    X=copy.deepcopy(ps)
    X[:,2]=1
    Z=ps[:,2]
    temp=np.dot(X.T,X)
    dt=np.linalg.det(temp)
    if dt!=0:
        A=np.dot(np.dot(np.linalg.inv(temp),X.T),Z)
        a=A[0]
        b=A[1]
        c=A[2]
        return a*p[0]+b*p[1]+c
    else:
        return None


def edges_to_lineset(mesh, edges, color):
    ls = o3d.geometry.LineSet()#边缘转线集
    ls.points = mesh.vertices
    ls.lines = edges
    # print(np.asarray(edges))
    colors = np.empty((np.asarray(edges).shape[0], 3))
    colors[:] = color
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls

def reverse_coordinate(pcd,A,offset,theta):
    a=A[0]
    b=A[1]
    c=A[2]
    pcdc=copy.deepcopy(pcd)
    # xyz=np.asarray(pcdc.points)
#     for i in range(xyz.shape[0]):
#         xyz[i][2]=xyz[i][2]+z
# #         print(xyz[i])
#         xyz[i]=xyz[i]-offset
# #         print(xyz[i])
# #     print(xyz)
    # pcdc.points=o3d.utility.Vector3dVector(xyz)
    # xyz=np.asarray(pcdc.points)    
    axis=np.array([[b/(a**2+b**2)**0.5],[-a/(a**2+b**2)**0.5],[0]])
    angle=1*math.acos(1/(a**2+b**2+1)**0.5)
    aa=np.dot(axis,angle)
    R2reverse=pcdc.get_rotation_matrix_from_zyx((-theta,0,0))
    pcdc.rotate(R2reverse,center=(0,0,0))
    Rreverse=pcdc.get_rotation_matrix_from_axis_angle(aa)
    pcdc.rotate(Rreverse,center=(0,0,0))
    xyz=np.asarray(pcdc.vertices)
    for i in range(xyz.shape[0]):
        xyz[i]=xyz[i]+offset
    pcdc.vertices=o3d.utility.Vector3dVector(xyz)
    return pcdc
#%%
print('loading pcd')
# pcd=o3d.io.read_triangle_mesh(filename)
# xyz=np.asarray(pcd.vertices)
# o3d.visualization.draw_geometries([pcd],mesh_show_wireframe=True,mesh_show_back_face=True)
pcdo=o3d.geometry.TriangleMesh()

plydata = PlyData.read(filename)
if 'face' in plydata:
    if len(plydata['face']['vertex_indices'])>0:
        pcdo=o3d.geometry.TriangleMesh()
        plytype='trianglemesh'
        mesh=plydata['face']['vertex_indices']
        pcdo.triangles=o3d.utility.Vector3iVector(mesh)
    else:
        pcdo=o3d.geometry.PointCloud()
        plytype='pointcloud'
else:
    pcdo=o3d.geometry.PointCloud()
    plytype='pointcloud'
xlist = plydata['vertex']['x']
ylist = plydata['vertex']['y']
zlist = plydata['vertex']['z']
xyzo=np.array([xlist,ylist,zlist])
xyzo=xyzo.transpose()
mean=np.mean(xyzo,axis=0)

for i in range(xyzo.shape[0]):
    xyzo[i]=xyzo[i]-mean

if plytype=='pointcloud':
    rlist = plydata['vertex']['red']
    glist = plydata['vertex']['green']
    blist = plydata['vertex']['blue']
    rgbo=np.array([rlist,glist,blist])
    rgbo=rgbo.transpose()/255
    pcdo.colors=o3d.utility.Vector3dVector(rgbo)
    pcdo.points=o3d.utility.Vector3dVector(xyzo)
else:
    pcdo.vertices=o3d.utility.Vector3dVector(xyzo)

#%%
print('rigid ransforming')
#平面拟合,旋转
X=copy.deepcopy(xyzo)
X[:,2]=1
Z=xyzo[:,2]
          
A=np.dot(np.dot(np.linalg.inv(np.dot(X.T,X)),X.T),Z)
a=A[0]
b=A[1]
c=A[2]

axis=np.array([[b/(a**2+b**2)**0.5],[-a/(a**2+b**2)**0.5],[0]])
angle=-1*math.acos(1/(a**2+b**2+1)**0.5)
aa=np.dot(axis,angle)

pcd1=copy.deepcopy(pcdo)
R=pcd1.get_rotation_matrix_from_axis_angle(aa)
pcd1.rotate(R,center=(0,0,0))


# o3d.visualization.draw_geometries([pcd,pcd1],mesh_show_wireframe=True,mesh_show_back_face=True)


#%%
#点云投影转图片
print('projection')
if plytype=='trianglemesh':
    resolution=[300,400]
    xyz1=np.asarray(pcd1.vertices)
else:
    resolution=[300,400,3]
    xyz1=np.asarray(pcd1.points)
    rgb1=np.asarray(pcd1.colors)
margin=50
# xyz=np.asarray(pcd1.points)
# rgb=np.asarray(pcd1.colors)
minx=np.min(xyz1[:,0])
maxx=np.max(xyz1[:,0])
miny=np.min(xyz1[:,1])
maxy=np.max(xyz1[:,1])
dx=maxx-minx
dy=maxy-miny
if dx/dy>=(resolution[1]-2*margin)/(resolution[0]-2*margin):
    dp=dx/(resolution[1]-2*margin)
else:
    dp=dy/(resolution[0]-2*margin)
projectionimg=np.zeros(resolution)
# for i in range(len(xyz1)):
#     p=xyz1[i]
#     # c=rgb[i]*255
#     # c=c.astype(int)
#     pixelx=int((p[0]-minx)/dp)-1+margin
#     pixely=int((maxy-p[1])/dp)-1+margin
#     projectionimg[pixely][pixelx]=c
projectionimg=projectionimg.astype(np.uint8)

# img_gray = cv2.cvtColor(projectionimg,cv2.COLOR_BGR2GRAY)
# cv2.imwrite(path+'/temp.png',projectionimg)
# cv2.imwrite(path+'/gray.png',img_gray)

#%%
#提取网格边界
print('extract edges')
if plytype=='trianglemesh':
    edge_manifold_boundary = pcd1.is_edge_manifold(allow_boundary_edges=False)
    if not edge_manifold_boundary:
        edges = pcd1.get_non_manifold_edges(allow_boundary_edges=False)
        # geoms = [pcd1]
        # geoms.append(edges_to_lineset(pcd1, edges, (0, 1, 0)))
        # o3d.visualization.draw_geometries(geoms, mesh_show_back_face=True)
        
        edges_arr=np.asarray(edges)
        coords_arr=[]
        for i in range(len(edges_arr)):
            e=edges_arr[i]
            pixelx1=int((xyz1[e[0]][0]-minx)/dp)-1+margin
            pixely1=int((maxy-xyz1[e[0]][1])/dp)-1+margin
            pixelx2=int((xyz1[e[1]][0]-minx)/dp)-1+margin
            pixely2=int((maxy-xyz1[e[1]][1])/dp)-1+margin
            coords=[pixelx1,pixely1,pixelx2,pixely2]
            coords_arr.append(coords)
        coords_arr=np.asarray(coords_arr)
        # print(coords_arr)
        for x1, y1, x2, y2 in coords_arr:
            cv2.line(projectionimg, (x1, y1), (x2, y2), 255, 2)
            # print((x1, y1), (x2, y2))
            cv2.imwrite(path+'/temp.png',projectionimg)
            
elif plytype=='pointcloud':
    for i in range(len(xyz1)):
        p=xyz1[i]
        c=rgbo[i]*255
        pixelx=int((p[0]-minx)/dp)-1+margin
        pixely=int((maxy-p[1])/dp)-1+margin
        projectionimg[pixely][pixelx]=c
    projectionimg=projectionimg.astype(np.uint8) 
    
    img_gray = cv2.cvtColor(projectionimg,cv2.COLOR_BGR2GRAY)
    maskindex=np.where(img_gray==0)
    mask=np.zeros(resolution[:-1])
    mask[maskindex]=255
    mask=mask.astype(np.uint8)
    dst= cv2.inpaint(projectionimg,mask,3,cv2.INPAINT_TELEA)
    img1 = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
    img2=copy.deepcopy(img1)
    cimg=copy.deepcopy(dst)
    canny = cv2.Canny(img2, 100, 100)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    closed_canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE,kernel)
    
    lines = cv2.HoughLinesP(closed_canny,1,np.pi/72,50,minLineLength=5,maxLineGap=10)
    for i in range(len(lines)):
        for x1, y1, x2, y2 in lines[i]:
            cv2.line(closed_canny, (x1, y1), (x2, y2), 255, 2)
    projectionimg = cv2.morphologyEx(closed_canny, cv2.MORPH_CLOSE,kernel)
    cv2.imwrite(path+'/temp.png',projectionimg)

#%%
cimg=copy.deepcopy(projectionimg)
contours,hierarchy = cv2.findContours(projectionimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

la=[]
MA=[]
for i in range(len(contours)):
    area=cv2.contourArea(contours[i])
    la.append([i,area])
la=sorted(la,key=lambda x:x[1],reverse=True)
cnt=contours[la[0][0]]

coef=1
edgenumber=3
 
# while edgenumber<4:
#     coef*=0.95
#     if coef<1e-3:
#         raise ValueError
#     epsilon=coef*cv2.arcLength(cnt,True)
#     approx = cv2.approxPolyDP(cnt,epsilon,True)
#     edgenumber=approx.shape[0]
# if edgenumber==4:
#     V=approx[:,0,:]
# else:
#     print('2')
#     rect = cv2.minAreaRect(cnt)
#     box = cv2.boxPoints(rect)  
#     box = np.int0(box)
#     V=box

rect = cv2.minAreaRect(cnt)
box = cv2.boxPoints(rect)  
box = np.int0(box)
V=box

rect_center=np.mean(box,axis=0)
rect_off=box-rect_center
cnt_off=cnt-rect_center
cnt_off=np.float32(cnt_off)

# inrect=False
# scale=1
# while not inrect:
#     rect_scale=rect_off*scale
#     templ=[]
#     for pt in rect_scale:
#         pi=cv2.pointPolygonTest(cnt_off,pt, False)
#         # print(pi)
#         templ.append(pi)
#     if min(templ)<0:
#         scale-=0.05
#         if scale<0:
#             # raise ValueError
#             break
#     else:
#         inrect=True
rect_scale=rect_off*0.9
V=rect_scale+rect_center
V=np.int64(V)

cv2.drawContours(cimg,[V],-1,125,3)  
cv2.imwrite(path+'/edge detection.png',cimg)

#%%
# imgcopy=copy.deepcopy(projectionimg)
# cv2.drawContours(imgcopy,cnt,-1,125,3)  
# # cv2.imshow('rect',projectionimg)
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()
# cv2.imshow('img2',dst)
# cv2.waitKey()
# cv2.imshow('cimg',closed_canny)
# cv2.waitKey()
# cv2.destroyAllWindows()
#%%
# cimg2=copy.deepcopy(projectionimg)
# epsilon=0.01*cv2.arcLength(cnt,True)
# approx=cv2.approxPolyDP(cnt,epsilon,True)
# V=approx[:,0,:]

# cv2.drawContours(cimg2,[V],-1,120,3)  
# cv2.imshow('rect',cimg2)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

#%%
#像素转坐标
print('pixels to coords')
midx=0
midy=0
for i in range(4):
    midx+=V[i][0]/4
    midy+=V[i][1]/4
midx=(midx+1-margin)*dp+minx
midy=maxy-(midy+1-margin)*dp
Vertex=[[],[],[],[]]
for v in V:
    t=[(v[0]+1-margin)*dp+minx,maxy-(v[1]+1-margin)*dp,0]
    if t[0]<=midx and t[1]>=midy:
        Vertex[0]=t
    elif t[0]>midx and t[1]>midy:
        Vertex[1]=t
    elif t[0]>=midx and t[1]<=midy:
        Vertex[2]=t
    elif t[0]<midx and t[1]<midy:
        Vertex[3]=t
# print(Vertex)
pcdv=o3d.geometry.PointCloud()
pcdv.points=o3d.utility.Vector3dVector(np.array(Vertex))
#旋转点云，摆正边缘

ti=[]
for i in range(4):
    direction=[Vertex[i%4][0]-Vertex[(i+1)%4][0],Vertex[i%4][1]-Vertex[(i+1)%4][1]]
    if direction[0]!=0:
        t=-1*direction[1]/direction[0]
    else:
        t=float('inf')
    ti.append(t)

theta=math.atan(ti[0])
# print(t1,t2,t3,t4)
# print(direction1)
# print(theta)

pcd2=o3d.geometry.PointCloud()
pcd2.points=o3d.utility.Vector3dVector(xyz1)
pcd2.colors=o3d.utility.Vector3dVector(rgb1)
# o3d.visualization.draw_geometries([pcdv])

R1=pcd2.get_rotation_matrix_from_zyx((theta,0,0))
pcd2.rotate(R1,center=(0,0,0))
# o3d.visualization.draw_geometries([pcd2])

xyz2=np.asarray(pcd2.points)


R2=pcdv.get_rotation_matrix_from_zyx((theta,0,0))
pcdv.rotate(R2,center=(0,0,0))
Vertex2=np.asarray(pcdv.points)[:,:2]

# margin=(max(Vertex2[:,0])-min(Vertex2[:,0]))*0.05
margin=0
ind1=np.where((xyz2[:,0]>min(Vertex2[:,0])+margin) & (xyz2[:,0]<max(Vertex2[:,0])-margin))
temp=xyz2[ind1]
tempc=rgb1[ind1]
ind2=np.where((temp[:,1]>min(Vertex2[:,1])+margin) & (temp[:,1]<max(Vertex2[:,1])-margin))
xyz3=temp[ind2]
rgb3=tempc[ind2]

scalefactor=(np.max(xyz3,axis=0)[0]-np.min(xyz3,axis=0)[0])/100
xyz3=xyz3/scalefactor
# xyz3=xyz3/1

pcd3=o3d.geometry.PointCloud()
pcd3.points=o3d.utility.Vector3dVector(xyz3)
pcd3.colors=o3d.utility.Vector3dVector(rgb3)
#%%
# o3d.visualization.draw_geometries([pcd3])
#%%
#网格采样
print('mesh sampling')
# cellsize=0.008
# xlim=[min(xyz3[:,0]),max(xyz3[:,0])]
# ylim=[min(xyz3[:,1]),max(xyz3[:,1])]
# nx=int((xlim[1]-xlim[0])/cellsize)+1
# ny=int((ylim[1]-ylim[0])/cellsize)+1
mx0=80

xlim=[min(xyz3[:,0]),max(xyz3[:,0])]
ylim=[min(xyz3[:,1]),max(xyz3[:,1])]
nx=mx0
cellsize=(xlim[1]-xlim[0])/nx
ny=int((ylim[1]-ylim[0])/cellsize)+1
size=np.array((nx,ny))

imgs=np.zeros((nx,ny),dtype=np.uint8)
minz=np.min(xyz3,axis=0)[2]
maxz=np.max(xyz3,axis=0)[2]
dz=(maxz-minz)/255
masks=np.zeros((nx,ny),dtype=np.uint8)
indexs=np.zeros((nx*ny))
#%%
sample_list=[]
sample_list_rgb=[]
sample_index=[]
# print(nx)
for i in range(nx):
    # print(i)
    ind1=np.where((xyz3[:,0]>=xlim[0]+i*cellsize)&(xyz3[:,0]<xlim[0]+(i+1)*cellsize))
    temp1=xyz3[ind1]
    if len(temp1)!=0:
        counter=0
        for j in range(ny):
            ind2=np.where((temp1[:,1]>=ylim[0]+j*cellsize)&(temp1[:,1]<ylim[0]+(j+1)*cellsize))
            temp2=temp1[ind2]
            if len(temp2)!=0:
                cx=xlim[0]+(i+0.5)*cellsize
                cy=ylim[0]+(j+0.5)*cellsize
                mid=[cx,cy]
                D=float('inf')
                P=None
                for r in range(len(temp2)):
                    p=temp2[r].tolist()
                    d=(p[0]-mid[0])**2+(p[1]-mid[1])**2
                    if d<D:
                        D=d
                        P=p
                cz=P[2]
                Pm=[cx,cy,cz]
                imgs[i][j]=int((cz-minz)/dz)
                counter+=1
                sample_list.append(Pm)
                sample_index.append([i,j])
                sample_list_rgb.append([255,0,0])
            else:
                cx=xlim[0]+(i+0.5)*cellsize
                cy=ylim[0]+(j+0.5)*cellsize
                temp=[]
                cz=None
                t=1
                while len(temp)<4:
                    t+=1
                    temp=xyz3[(abs(xyz3[:,0]-cx)<cellsize*t) & (abs(xyz3[:,1]-cy)<cellsize*t)]
                    # temp=xyz3[0.2*(xyz3[:,0]-cx)**2+(xyz3[:,1]-cy)**2<(t*cellsize)**2]
                #     cz=planefit([cx,cy],temp)
                
                X=copy.deepcopy(temp)
                X[:,2]=1
                Z=temp[:,2]
                A=np.dot(np.dot(np.linalg.inv(np.dot(X.T,X)),X.T),Z)
                a=A[0]
                b=A[1]
                c=A[2]
                cz=a*cx+b*cy+c
                # temp=xyz2[(abs(xyz2[:,0]-cx)<cellsize*2) & (abs(xyz2[:,1]-cy)<cellsize*8)]
                # temp=[]
                # cz=None
                # t=3
                # while cz==None:
                #     t+=1
                #     temp=xyz3[(abs(xyz3[:,0]-cx)<cellsize*(t)) & (abs(xyz3[:,1]-cy)<cellsize*t)]
                #     # temp=xyz3[0.2*(xyz3[:,0]-cx)**2+(xyz3[:,1]-cy)**2<(t*cellsize)**2]
                #     if len(temp)>1:
                #     #     cz=planefit([cx,cy],temp)
                #         cz=np.mean(temp,axis=0)[2]
                #         break
                #print(i,cx)
                mid=[cx,cy,cz]
                
                imgs[i][j]=int((cz-minz)/dz)
                # masks[i][j]=1
                indexs[i*ny+j]=1
                #mid_rgb=[255,255,255]
                counter+=1
                sample_list.append(mid)
                sample_list_rgb.append([0,255,0])
                sample_index.append([i,j])
                #print(i)
        #print(counter)

#%%
#计算采样点曲率
#zx,nx,ny
print('filtering')
xyz4=np.array(sample_list)
# rgb4=np.array(sample_list_rgb)
index=np.array(sample_index)

# imgs=np.array(imgs,dtype=np.uint8)
# imgs1=cv2.GaussianBlur(imgs,(3,3),0) #高斯滤波

offset_imgs=np.min(imgs)

minz=np.min(xyz4,axis=0)[2]
maxz=np.max(xyz4,axis=0)[2]
dz=(maxz-minz)/255
for i in range(nx):
    for j in range(ny):
        cz=xyz4[i*ny+j][2]
        imgs[i][j]=int((cz-minz)/dz)
imgs1=cv2.medianBlur(imgs,5)
imgs1=cv2.bilateralFilter(imgs1,7,40,40) #双边滤波
# cv2.imwrite(path+'/m_scratch.png',r_scratch)
imgs2=imgs1*dz+np.ones((nx,ny))*minz
xyz4c=copy.deepcopy(xyz4)
# replace=imgs2[np.where(masks==1)]
place=np.where(indexs==1)
# for i in range(len(indexs)):
#     xyz4c[place[0][i]]
# xyz4c[np.where(indexs==1)[0],2]=replace
for i in range(nx):
    for j in range(ny):
        xyz4c[i*ny+j][2]=imgs2[i][j]
# cv2.imshow('gb', imgs)
# cv2.waitKey(0)
# cv2.imshow('gb', imgs1)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
cv2.imwrite(path+'/beforeGB.png',imgs)
cv2.imwrite(path+'/afterGB.png',imgs1)

#%%
print('calculating curvature')
# xyz4c=copy.deepcopy(sample_list)
zxl=[]
zyl=[]
zxxl=[]
zyyl=[]
zxyl=[]
for i in range(len(xyz4c)):
    if i+ny<=len(xyz4c)-1:
        zx=xyz4c[i+ny][2]-xyz4c[i][2]
    else:
        zx=xyz4c[i][2]-xyz4c[i-ny][2]
    if (i+1)%ny!=0:
        # print(i)
        zy=xyz4c[i+1][2]-xyz4c[i][2]
    else:
        zy=xyz4c[i][2]-xyz4c[i-1][2]
    zxl.append(zx)
    zyl.append(zy)
for i in range(len(xyz4c)):
    if i+ny<=len(xyz4c)-1:
        zxx=zxl[i+ny]-zxl[i]
    else:
        zxx=zxl[i]-zxl[i-ny]
    if (i+1)%ny!=0:
        zyy=zyl[i+1]-zyl[i]
        zxy=zxl[i+1]-zxl[i]
    else:
        zyy=zyl[i]-zyl[i-1]
        zxy=zxl[i]-zxl[i-1]
    zxxl.append(zxx)
    zyyl.append(zyy)
    zxyl.append(zxy)
zx=np.asarray(zxl)
zy=np.asarray(zyl)
zxx=np.asarray(zxxl)
zyy=np.asarray(zyyl)
zxy=np.asarray(zxyl)


ones=np.ones((nx*ny))
ave_curvature=abs(((ones+zx*zx)*zyy-2*zx*zy*zxy+(ones+zy*zy)*zxx)/(2*(ones+zx*zx+zy*zy)**(3/2)))
scratch_label=copy.deepcopy(ave_curvature)

# scratch_label=scratch_label/np.max(scratch_label)


index=10
ref=0.2
thre=50

# scratch_label[np.where(scratch_label>=thre)]=1
# scratch_label[np.where(scratch_label<thre)]=0
# print(np.max(scratch_label))
scratch_label=scratch_label/ref
# scratch_label=scratch_label/np.max(scratch_label)


scratch_label=1.0/(1+np.exp(-1*index*(scratch_label-0.5)))



# scratch_label[np.where(scratch_label>=thre)]=1
# scratch_label[np.where(scratch_label<thre)]=0

r_curvature=ave_curvature.reshape(nx,ny)
r_scratch=scratch_label.reshape(nx,ny).transpose()*255
r_scratch= np.array(r_scratch,np.uint8)

cv2.imwrite(path+'/o_scratch.png',r_scratch)

kernel=np.ones((3,3),np.uint8)
r2_scratch=cv2.dilate(r_scratch,kernel)
cv2.imwrite(path+'/r2_scratch.png',r2_scratch)
# ret,r3_scratch=cv2.threshold(r2_scratch,thre,255,cv2.THRESH_BINARY)
ret,r3_scratch=cv2.threshold(r_scratch,thre,255,cv2.THRESH_BINARY)
# scrath_pixel=np.where()
kernel1=np.ones((3,3),np.uint8)
kernel2=np.ones((5,5),np.uint8)
r3_scratch=cv2.dilate(r3_scratch,kernel1) 
r3_scratch=cv2.erode(r3_scratch,kernel2)
scratchmask=np.where(r3_scratch==255)

cv2.imwrite(path+'/r3_scratch.png',r3_scratch)

r_scratch=cv2.Canny(r3_scratch,200,300)
cv2.imwrite(path+'/c_scratch.png',r_scratch)


# cv2.imshow('r2_scratch',r2_scratch)6
# cv2.imshow('r3_scratch',r3_scratch)
# cv2.waitKey()
# cv2.destroyAllWindows()



#%%
# pcd4t=o3d.geometry.PointCloud()
# pcd4t.points=o3d.utility.Vector3dVector(xyz4c)
# pcd4t.colors=o3d.utility.Vector3dVector(sample_list_rgb)
# o3d.visualization.draw_geometries([pcd4t],mesh_show_wireframe=True,mesh_show_back_face=True)

#%%
#标记折痕和端点
print('labeling scratch')
kernel1=np.ones((3,3),np.uint8)
kernel2=np.ones((3,3),np.uint8)
for i in range(1):
    r_scratch=cv2.dilate(r_scratch,kernel1) 
for i in range(1):
    r_scratch=cv2.erode(r_scratch,kernel2)
if densify:
    slines=cv2.HoughLinesP(r_scratch,1,np.pi/180,50,minLineLength=int(min(nx,ny)/5),maxLineGap=int(min(nx,ny)/8))
else:
    slines=None
# print(slines)
#%%
# print(slines)
if slines is None:
    print('No scratch')
    slines=np.array([])

# print(slines)

#%%
# if densify:
directionlist,distancelist,parallelpair,slines,indexlist=findparallel(slines,dcri=5)
# print(slines)

# r_scratch=r_scratch.astype(np.uint8).copy()
scratchimg=np.zeros(r_scratch.shape)
for l in slines:
    for x1, y1, x2, y2 in l:
        cv2.line(scratchimg, (x1, y1), (x2, y2),255, 1)
# cv2.imshow('cratch',r_scratch*255)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
cv2.imwrite(path+'/r_scratch.png',r_scratch)
cv2.imwrite(path+'/scratch.png',scratchimg)

nx=mx
cellsize=(xlim[1]-xlim[0])/nx
ny=int((ylim[1]-ylim[0])/cellsize)+1


slines=slines/mx0*mx
slines=slines.astype(np.uint8)
#%%
bondarylines=[[0,0,nx-1,0],[nx-1,0,nx-1,ny-1],[nx-1,ny-1,0,ny-1],[0,ny-1,0,0]]
cptlist=[]
for l in slines:
    cptl=[]
    for b in bondarylines:
        cpt=cross_point(l[0],b)
        # print(cpt)
        if not np.nan in cpt:
            if cpt[0]>=0 and cpt[0]<=nx-1 and cpt[1]>=0 and cpt[1]<=ny-1:
                cptl+=cpt
    # print(cptl)
    # print(cptl)
    cptlist.append(cptl)
#%%
#seeding
print('seeding')
num=int(nx*0.2)
msize=0.4
l1=[0,ny-1]
l2=[0,nx-1]
l3=[0,ny-1]
l4=[0,nx-1]
for p in cptlist:
    for i in range(int(len(p)/2)):
        pt=p[i*2:i*2+2]
        # print(pt)
        if pt[0]==0:
            l1.append(pt[1])
        elif pt[0]==nx-1:
            l3.append(pt[1])
        if pt[1]==0:
            l2.append(pt[0])
        elif pt[1]==ny-1:
            l4.append(pt[0])
l1=list(set(l1))
l2=list(set(l2))
l3=list(set(l3))
l4=list(set(l4))
l1.sort()
l2.sort()
l3.sort()
l4.sort()
seedlist0=[l1,l2,l3,l4]
seedlist=[[],[],[],[]]
for j in range(len(seedlist0)):
    l=seedlist0[j]
    if len(l)>3:
        seedlist[j].append(l[0])
        for i in range(1,len(l)-2):
            s=l[i]
            # print(l[i+1]-s)
            if l[i+1]-s<5*mx/mx0:
                seedlist[j].append(0.5*(l[i+1]+s))
            else:
                seedlist[j]+=[l[i],l[i+1]]
        # print(seedlist)
        seedlist[j].append(l[-1])
    else:
        seedlist[j]=l

# print(seedlist)

seedlist1=[]
for l in seedlist:
    li=copy.deepcopy(l)
    for i in range(1,len(l)-1):
        # print(i)
        s=l[i]
        if s-l[i-1]<=num*msize and l[i+1]-s>num*msize:
            la1=np.linspace(l[i-1],s,num+1).tolist()
            la2=np.linspace(s,s+num*msize,num+1).tolist()
        elif s-l[i-1]<=num*msize and l[i+1]-s<=num*msize:
            la1=np.linspace(l[i-1],s,num+1).tolist()
            la2=np.linspace(s,l[i+1],num+1).tolist()
        elif s-l[i-1]>num*msize and l[i+1]-s<=num*msize:
            la1=np.linspace(s-num*msize,s,num+1).tolist()
            la2=np.linspace(s,l[i+1],num+1).tolist()
        else:
            la1=np.linspace(s-num*msize,s,num+1).tolist()
            la2=np.linspace(s,s+num*msize,num+1).tolist()
        li+=la1+la2
        # print(li)
    # print(l)
    if len(l)==2:
        li=copy.deepcopy(l)
    li=list(set(li))
    # print(li)
    li.sort()
    # print(li)
    seedlist1.append(li)
seedlist2=[[],[],[],[]]

# print(seedlist1)

if len(l1)>=len(l3):
    for i in range(len(seedlist1[0])-1):
        li=np.linspace(seedlist1[0][i],seedlist1[0][i+1],int(seedlist1[0][i+1]-seedlist1[0][i]+1)).tolist()
        seedlist2[0]+=li
    seedlist2[0]=list(set(seedlist2[0]))
    mnum=len(seedlist2[0])
    space=[]
    vs=[]
    for i in range(len(seedlist1[2])-1):
        s=seedlist1[2][i+1]-seedlist1[2][i]
        if s>1:
            space.append(s)
            vs.append([seedlist1[2][i],seedlist1[2][i+1]])
    n=mnum-len(seedlist1[2])
    nl=n
    for i in range(len(space)):
        # print(ni)
        if i!=len(space)-1:
            s=space[i]
            ni=int(s/sum(space)*n)
            nl-=ni
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        else:
            ni=nl
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        seedlist2[2]+=li
    seedlist2[2]+=seedlist1[2]
    seedlist2[2]=list(set(seedlist2[2]))
    seedlist2[2].sort()
    seedlist2[0].sort()
else:
    for i in range(len(seedlist1[2])-1):
        li=np.linspace(seedlist1[2][i],seedlist1[2][i+1],int(seedlist1[2][i+1]-seedlist1[2][i]+1)).tolist()
        seedlist2[2]+=li
    seedlist2[2]=list(set(seedlist2[2]))
    mnum=len(seedlist2[2])
    space=[]
    vs=[]
    for i in range(len(seedlist1[0])-1):
        s=seedlist1[0][i+1]-seedlist1[0][i]
        if s>1:
            space.append(s)
            vs.append([seedlist1[0][i],seedlist1[0][i+1]])
    n=mnum-len(seedlist1[0])
    nl=n
    for i in range(len(space)):
        # print(ni)
        if i!=len(space)-1:
            s=space[i]
            ni=int(s/sum(space)*n)
            nl-=ni
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        else:
            ni=nl
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        seedlist2[0]+=li
    seedlist2[0]+=seedlist1[0]
    seedlist2[0]=list(set(seedlist2[0]))
    seedlist2[0].sort()
    seedlist2[2].sort()
if len(l2)>=len(l4):
    for i in range(len(seedlist1[1])-1):
        li=np.linspace(seedlist1[1][i],seedlist1[1][i+1],int(seedlist1[1][i+1]-seedlist1[1][i]+1)).tolist()
        seedlist2[1]+=li
    seedlist2[1]=list(set(seedlist2[1]))
    mnum=len(seedlist2[1])
    space=[]
    vs=[]
    for i in range(len(seedlist1[3])-1):
        s=seedlist1[3][i+1]-seedlist1[3][i]
        if s>1:
            space.append(s)
            vs.append([seedlist1[3][i],seedlist1[3][i+1]])
    n=mnum-len(seedlist1[3])
    nl=n
    for i in range(len(space)):
        # print(ni)
        if i!=len(space)-1:
            s=space[i]
            ni=int(s/sum(space)*n)
            nl-=ni
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        else:
            ni=nl
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        seedlist2[3]+=li
    seedlist2[3]+=seedlist1[3]
    seedlist2[3]=list(set(seedlist2[3]))
    seedlist2[3].sort()
    seedlist2[1].sort()
else:     
    for i in range(len(seedlist1[3])-1):
        li=np.linspace(seedlist1[3][i],seedlist1[3][i+1],int(seedlist1[3][i+1]-seedlist1[3][i]+1)).tolist()
        seedlist2[3]+=li
    seedlist2[3]=list(set(seedlist2[3]))
    mnum=len(seedlist2[3])
    space=[]
    vs=[]
    for i in range(len(seedlist1[1])-1):
        s=seedlist1[1][i+1]-seedlist1[1][i]
        if s>1:
            space.append(s)
            vs.append([seedlist1[1][i],seedlist1[1][i+1]])
    n=mnum-len(seedlist1[1])
    nl=n
    for i in range(len(space)):
        # print(ni)
        if i!=len(space)-1:
            s=space[i]
            ni=int(s/sum(space)*n)
            nl-=ni
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        else:
            ni=nl
            li=np.linspace(vs[i][0],vs[i][1],ni+2).tolist()
        seedlist2[1]+=li
    seedlist2[1]+=seedlist1[1]
    seedlist2[1]=list(set(seedlist2[1]))
    seedlist2[1].sort()
    seedlist2[3].sort()
#%%
#meshing
print('meshing')
nmx=len(seedlist2[1])
nmy=len(seedlist2[0])
size=np.array((nmx,nmy))


masks=np.zeros((nmx,nmy),dtype=np.uint8)
indexs=np.zeros((nmx*nmy))

sample_list=[]
sample_list_rgb=[]
sample_index=[]

for i in range(nmx):
    for j in range(nmy):
        l1=[0,seedlist2[0][j],nx,seedlist2[2][j]]
        l2=[seedlist2[1][i],0,seedlist2[3][i],ny]
        pt=cross_point(l1,l2)
        
        cx=xlim[0]+(pt[0]+0.5)*cellsize
        cy=ylim[0]+(pt[1]+0.5)*cellsize
        # print(pt)
        # [cx,cy]
        # print(cx,cy)
        ind1=np.where((xyz3[:,0]>=cx-0.5*cellsize)&(xyz3[:,0]<cx+0.5*cellsize))
        temp1=xyz3[ind1]

        if len(temp1)!=0:
            counter=0
            ind2=np.where((temp1[:,1]>=cy-0.5*cellsize)&(temp1[:,1]<cy+0.5*cellsize))
            temp2=temp1[ind2]
            # print(len(temp2))
            if len(temp2)!=0:
                # cx=xlim[0]+(i+0.5)*cellsize
                # cy=ylim[0]+(j+0.5)*cellsize
                mid=[cx,cy]
                D=float('inf')
                P=None
                for r in range(len(temp2)):
                    p=temp2[r].tolist()
                    d=(p[0]-mid[0])**2+(p[1]-mid[1])**2
                    if d<D:
                        D=d
                        P=p
                cz=P[2] 
                sample_list_rgb.append([255,0,0])
            else:
                # cx=xlim[0]+(i+0.5)*cellsize
                # cy=ylim[0]+(j+0.5)*cellsize
                # temp=xyz2[(abs(xyz2[:,0]-cx)<cellsize*2) & (abs(xyz2[:,1]-cy)<cellsize*8)]
                temp=[]
                cz=None
                t=1
                while len(temp)<4:
                    t+=1
                    temp=xyz3[(abs(xyz3[:,0]-cx)<cellsize*t) & (abs(xyz3[:,1]-cy)<cellsize*t)]
                    # temp=xyz3[0.2*(xyz3[:,0]-cx)**2+(xyz3[:,1]-cy)**2<(t*cellsize)**2]
                #     cz=planefit([cx,cy],temp)
                
                X=copy.deepcopy(temp)
                X[:,2]=1
                Z=temp[:,2]
                A=np.dot(np.dot(np.linalg.inv(np.dot(X.T,X)),X.T),Z)
                a=A[0]
                b=A[1]
                c=A[2]
                cz=a*cx+b*cy+c
                
                sample_list_rgb.append([0,255,0])
                #print(i,cx)
                mid=[cx,cy,cz]
                # mid_rgb=[0,0,0]
                #mid_rgb=[255,255,255]
                # sample_list_rgb.append(mid_rgb)
            mid=[cx,cy,cz]
            sample_list.append(mid)
            masks[i][j]=1
            indexs[i*ny+j]=1
            counter+=1
            sample_index.append([i,j])
xyz4=np.array(sample_list)
#%%
#coords of scratchmask
scratchlist=[]
scratchrgb=[]
print(len(scratchmask[0]))
for i in range(len(scratchmask[0])):
    xp=scratchmask[1][i]
    yp=scratchmask[0][i]
    
    xc=xlim[0]+(xp+0.5)*cellsize
    yc=ylim[0]+(yp+0.5)*cellsize
    
    ind1=np.where((xyz3[:,0]>=xc-0.5*cellsize)&(xyz3[:,0]<xc+0.5*cellsize))
    temp1=xyz3[ind1]
    if len(temp1)!=0:
        counter=0
        ind2=np.where((temp1[:,1]>=yc-0.5*cellsize)&(temp1[:,1]<yc+0.5*cellsize))
        temp2=temp1[ind2]
        # print(len(temp2))
        if len(temp2)!=0:
            # cx=xlim[0]+(i+0.5)*cellsize
            # cy=ylim[0]+(j+0.5)*cellsize
            mid=[xc,yc]
            D=float('inf')
            P=None
            for r in range(len(temp2)):
                p=temp2[r].tolist()
                d=(p[0]-mid[0])**2+(p[1]-mid[1])**2
                if d<D:
                    D=d
                    P=p
            zc=P[2] 
            scratchrgb.append([0,255,0])
            scratchlist.append([xc,yc,zc])
        else:
            # cx=xlim[0]+(i+0.5)*cellsize
            # cy=ylim[0]+(j+0.5)*cellsize
            # temp=xyz2[(abs(xyz2[:,0]-cx)<cellsize*2) & (abs(xyz2[:,1]-cy)<cellsize*8)]
            temp=[]
            cz=None
            t=1
            while len(temp)<4:
                t+=1
                temp=xyz3[(abs(xyz3[:,0]-xc)<cellsize*t) & (abs(xyz3[:,1]-yc)<cellsize*t)]
                # temp=xyz3[0.2*(xyz3[:,0]-cx)**2+(xyz3[:,1]-cy)**2<(t*cellsize)**2]
            #     cz=planefit([cx,cy],temp)
            
            X=copy.deepcopy(temp)
            X[:,2]=1
            Z=temp[:,2]
            A=np.dot(np.dot(np.linalg.inv(np.dot(X.T,X)),X.T),Z)
            a=A[0]
            b=A[1]
            c=A[2]
            zc=a*cx+b*cy+c
            
            scratchrgb.append([0,255,0])
            #print(i,cx)
            scratchlist.append([xc,yc,zc])
scratcharr=np.array(scratchlist)
scratchrgb=np.array(scratchrgb)
#%%
# pcds=o3d.geometry.TriangleMesh()
# pcds.vertices=o3d.utility.Vector3dVector(scratcharr)

pcds=o3d.geometry.PointCloud()
pcds.points=o3d.utility.Vector3dVector(scratcharr)
pcds.colors=o3d.utility.Vector3dVector(scratchrgb)
#%%
imgs=np.zeros((nmx,nmy),dtype=np.uint8)
# minz=np.min(xyz3,axis=0)[2]
# maxz=np.max(xyz3,axis=0)[2]
# dz=(maxz-minz)/255
minz=np.min(xyz4,axis=0)[2]
maxz=np.max(xyz4,axis=0)[2]
dz=(maxz-minz)/255
for i in range(nmx):
    for j in range(nmy):
        cz=xyz4[i*nmy+j][2]
        imgs[i][j]=int((cz-minz)/dz)
imgs1=cv2.bilateralFilter(imgs,3,20,20) #双边滤波
imgs1=cv2.medianBlur(imgs1,3)
# cv2.imwrite(path+'/m_scratch.png',r_scratch)
imgs2=imgs1*dz+np.ones((nmx,nmy))*minz
xyzf=copy.deepcopy(xyz4)
replace=imgs2[np.where(masks==1)]
# place=np.where(indexs==1)
# for i in range(len(indexs)):
#     xyzf[place[0][i]]
xyzf[np.where(indexs==1)[0],2]=replace
# cv2.imshow('gb', imgs)
# cv2.waitKey(0)
# cv2.imshow('gb', imgs1)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
cv2.imwrite(path+'/beforeGB.png',imgs)
cv2.imwrite(path+'/afterGB.png',imgs1)

#%%
xyz4=np.array(sample_list)
pcd4=o3d.geometry.PointCloud()
pcd4.points=o3d.utility.Vector3dVector(xyz4)
pcd4.colors=o3d.utility.Vector3dVector(sample_list_rgb)

# xyzf=copy.deepcopy(xyzf)
# for i in range(nmx):
#     for j in range(nmy):
#         xyzf[i*nmy+j][2]=imgs[i][j]
# xyzf[np.where(indexs==1)[0],2]=imgs2
pcdf=o3d.geometry.TriangleMesh()#o3d.geometry.PointCloud()
pcdf.vertices=o3d.utility.Vector3dVector(xyzf)



#%%
# pcdt=o3d.geometry.PointCloud()
# pcdt.points=o3d.utility.Vector3dVector(xyz4[:2*nmy])
# o3d.visualization.draw_geometries([pcdt])
#%%
# print(np.asarray(pcdf.vertices))
# o3d.visualization.draw_geometries([pcdf],mesh_show_wireframe=True,mesh_show_back_face=True)

# pcdf.colors=o3d.utility.Vector3dVector(rgb4)
pcdt=copy.deepcopy(pcdf)
xyzt=np.asarray(pcdt.vertices)

# o3d.visualization.draw_geometries([pcdf,pcd],mesh_show_wireframe=True,mesh_show_back_face=True)
R1i=np.linalg.inv(R1)
Ri=np.linalg.inv(R)
Rotation=np.dot(Ri,R1i)

pcdt.rotate(Rotation,center=(0,0,0))

Translation=np.array([[1,0,0,mean[0]],[0,1,0,mean[1]],[0,0,1,mean[2]],[0,0,0,1]])

#%%
print('saving')
element_list=[]
element_arr=[]
for m in range(nmx-1):
    for n in range(nmy-1):
        i=m*nmy+n
        element_u1=[i+1,i+2,i+nmy+2]
        element_u2=[i+1,i+nmy+2,i+nmy+1]
        # element_a=[ia[iu.index(i)] for i in element_u]
        element_list.append(element_u1)
        element_list.append(element_u2)
        
        element_a1=[i,i+1,i+nmy+1]
        element_a2=[i,i+nmy+1,i+nmy]
        element_arr.append(element_a1)
        element_arr.append(element_a2)

pcdf.triangles=o3d.utility.Vector3iVector(np.asarray(element_arr))

comma=','
text='*Node\n'
for i in range(len(xyz4)):
    row=sample_list[i]
    row=[str(m) for m in row]
    strrow=comma.join(row)
    text+=str(i+1)+','+strrow+'\n'
file=open(path+'/test0321.inp','w')
file.write(text)
file.close()

text='*Element, type=S3R\n'
for i in range(len(element_list)):
    row=element_list[i]
    row=[str(m) for m in row]
    strrow=comma.join(row)
    text+=str(i+1)+','+strrow+'\n'
file=open(path+'/test0321.inp','a')
file.write(text)
file.close()
file=open(path+'/element_list.txt','w')
file.write(text)
file.close()

file=open(path+'/params.txt','w')
file.write('node number:'+str(len(xyz4))+'\n')
file.write('element number:'+str(len(element_list))+'\n')
file.write('A:'+str(A)+'\n')
file.write('mean:'+str(mean)+'\n')
file.write('theta:'+str(theta)+'\n')
file.write('size:'+str(size)+'\n')
file.write('cellsize:'+str(cellsize)+'\n')
file.write('scalefactor:'+str(scalefactor)+'\n')
# file.write('Rotation:'+str(Rotation)+'\n')
# file.write('Translation:'+str(Translation)+'\n')
file.close()
#%%
interpcd_r=reverse_coordinate(pcdf,A,mean,theta)
interpcd_r_mesh=o3d.geometry.TriangleMesh()
interpcd_r_mesh.vertices=interpcd_r.vertices
interpcd_r_mesh.triangles=o3d.utility.Vector3iVector(np.asarray(element_arr))

# interpcd_s=reverse_coordinate(pcds,A,mean,theta)

o3d.io.write_triangle_mesh(path+'/interpcd_r.ply',interpcd_r_mesh,write_ascii= True)
o3d.io.write_triangle_mesh(path+'/interpcd.ply',pcdf,write_ascii= True)

o3d.io.write_point_cloud(path+'/interpcd_o.ply',pcd3,write_ascii= True)

o3d.io.write_point_cloud(path+'/interpcd_s.ply',pcds,write_ascii= True)
#%%
print('pre processing done')
#%%
# o3d.visualization.draw_geometries([interpcd_r_mesh,pcdf],mesh_show_wireframe=True,mesh_show_back_face=True)
# o3d.visualization.draw_geometries([interpcd_r_mesh,pcd],mesh_show_wireframe=True,mesh_show_back_face=True)
# o3d.visualization.draw_geometries([pcdf])
#%%
#%%
# pcdt=o3d.geometry.PointCloud()
# pcdt.points=o3d.utility.Vector3dVector(xyz4c)
# o3d.visualization.draw_geometries([pcdf,pcd4],mesh_show_wireframe=True,mesh_show_back_face=True)

# xyzt=np.asarray(pcdt.vertices)
# xyzt=xyzt.transpose()
# ones=np.ones((1,xyzt.shape[1]))

# xyzt=np.row_stack((xyzt,ones))
# xyzt=np.dot(Translation,xyzt)[:3].transpose()

# pcdt.vertices=o3d.utility.Vector3dVector(xyzt)

# o3d.visualization.draw_geometries([pcdo])

