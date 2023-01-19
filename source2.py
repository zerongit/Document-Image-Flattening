#%%
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import copy
import math
import time
import cv2
# import scipy.misc
import threadpool

from plyfile import *

etime=time.time()

f=open('./config.txt','r')
lines=f.readlines()
f.close()
conf=[]
for l in lines:
    r=l.split(',')[-1]
    conf.append(r)

path=conf[3][:-1]
path=path.replace('\\','/')
nx=int(conf[6])

patht=lines[2].split(',')[1][:-1]
# path=conf[3][:-1]
patht=patht.replace('\\','/')
patht=patht.strip('\r')
#%% 0413
#判断点在四边形中（投影）
def inquadrangle(point,element):
    il=[]
    for i in range(4):
        v1=np.array(point)-np.array(element[i])
        s1=np.array(element[(i+1)%4])-np.array(element[i])
        s2=np.array(element[i])-np.array(element[(i+3)%4])
        v1=v1[:2]
        s1=s1[:2]
        s2=s2[:2]
        indecator=np.dot(np.cross(v1,s1),np.cross(v1,s2))
        il.append(indecator)
    return il
#         if indecator<-1e-7:
#             return False
#     return True

#四边形逆双线性插值
def inverse_bilinear_interpolation(point,element_coord,element_displacement):
    d1=np.array(element_displacement[0])
    d2=np.array(element_displacement[1])
    d3=np.array(element_displacement[2])
    d4=np.array(element_displacement[3])
    c1=np.array(element_coord[0])[:2]
    c2=np.array(element_coord[1])[:2]
    c3=np.array(element_coord[2])[:2]
    c4=np.array(element_coord[3])[:2]
    
    e=c2-c1
    f=c4-c1
    g=c1-c2+c3-c4
    p=np.array(point)[:2]
    h=p-c1
    k2=np.cross(g,f)
    k1=np.cross(e,f)+np.cross(h,g)
    k0=np.cross(h,e)
    
    indecator=k1**2-4*k0*k2
    
    if indecator>=0:
        # inquatrangle=True
        # if abs(k2)>=0.05*abs(k1) or k2>1e-4: #0.2
        if abs(k2)>1e-6:
            label=np.array([255,0,255])
            w=indecator**0.5
            v=0.5*(-1*k1-w)/k2
            #print(h,f,e,g,v)
            if abs((e[0]+g[0]*v))>=abs((e[1]+g[1]*v)):
                u=(h[0]-f[0]*v)/(e[0]+g[0]*v)
            else:
                u=(h[1]-f[1]*v)/(e[1]+g[1]*v)
        else:
            v=-1*k0/k1
            if abs(e[0]*k1-g[0]*k0)>=abs(e[1]*k1-g[1]*k0):
                # print(1,k0,k1)
                label=np.array([0,255,0])
                u=(h[0]*k1+f[0]*k0)/(e[0]*k1-g[0]*k0)
            else:
                label=np.array([0,0,255])
                u=(h[1]*k1+f[1]*k0)/(e[1]*k1-g[1]*k0)
        dis1=d1+(d2-d1)*u
        dis2=d4+(d3-d4)*u
        dis=dis1+(dis2-dis1)*v
        return indecator,dis,label
    else:
        return indecator,0,np.array([0,0,0])
    # else:      
    #     v=-1*k0/k1
    #     if abs(e[0]*k1-g[0]*k0)>=abs(e[1]*k1-g[1]*k0):
    #         # print(2,k0,k1)
    #         label=np.array([0,255,0])
    #         u=(h[0]*k1+f[0]*k0)/(e[0]*k1-g[0]*k0)
    #     else:
    #         label=np.array([0,0,255])
    #         u=(h[1]*k1+f[1]*k0)/(e[1]*k1-g[1]*k0)
        

    # else:
        # # inquatrangle=False
        # dis=0
        # label=[255,255,0]
def triangle_interpolation(point,element_coord,element_displacement):
    x,y,z=point[0],point[1],point[2]
    x1,x2,x3=element_coord[0][0],element_coord[1][0],element_coord[2][0]
    y1,y2,y3=element_coord[0][1],element_coord[1][1],element_coord[2][1]
    d1,d2,d3=element_displacement[0],element_displacement[1],element_displacement[2]
    sa = 0.5*abs (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    s3 = 0.5*abs (x1 * (y2 - y) + x2 * (y - y1) + x * (y1 - y2))
    s2 = 0.5*abs (x1 * (y - y3) + x * (y3 - y1) + x3 * (y1 - y))
    s1 = 0.5*abs (x * (y2 - y3) + x2 * (y3 - y) + x3 * (y - y2))
    indecator=abs(s1 + s2 + s3 - sa)
    if indecator<0.0000001:
        dis=s1/sa*d1+s2/sa*d2+s3/sa*d3
        return True,dis
    else:
        return False,0
    # return abs(first + second + third - full) < .0000001

#网格采样
def homomesh(pcdc,nx=100,k=None):
#     cellsize=0.004
    pcd=copy.deepcopy(pcdc)
    if basedirection!=None:
        angle=math.atan(1/(1+k))
        R=pcd.get_rotation_matrix_from_zyx((-1*angle,0,0))
        pcd.rotate(R,center=(0,0,0))
        # o3d.visualization.draw_geometries([pcd,pcdc])
    xyz=np.asarray(pcd.points)
    xlim0=[min(xyz[:,0]),max(xyz[:,0])]
#     nx=int((xlim[1]-xlim[0])/cellsize)+1
    # nx=1600
    margin=6*(xlim0[1]-xlim0[0])/128
    xlim=[min(xyz[:,0])+margin,max(xyz[:,0])-margin]
    ylim=[min(xyz[:,1])+1*margin,max(xyz[:,1])-1*margin]
    cellsize=(xlim[1]-xlim[0])/nx
    ny=int((ylim[1]-ylim[0])/cellsize)
    sample_list=[]
    sample_list_rgb=[]
    sample_index=[]
    # print(nx,ny)
    indexlist=[]
    z=np.mean(xyz,axis=0)[2]
    for i in range(nx):
        for j in range(ny):
            mid=[xlim[0]+(i+0.5)*cellsize,ylim[0]+(j+0.5)*cellsize,z]
            sample_list.append(mid)
#             sample_list_rgb.append(C)
    xyznew=np.array(sample_list)
    pcdnew=o3d.geometry.PointCloud()
    pcdnew.points=o3d.utility.Vector3dVector(xyznew)
    if basedirection!=None:
        R2=pcdnew.get_rotation_matrix_from_zyx((angle,0,0))
        pcdnew.rotate(R,center=(0,0,0))
        # o3d.visualization.draw_geometries([pcdnew,pcdc])
        xyznew=np.asarray(pcdnew.points)
    return xyznew,pcdnew,(nx,ny)

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
    # print(Rreverse)
    pcdc.rotate(Rreverse,center=(0,0,0))
    
    xyz=np.asarray(pcdc.points)
    for i in range(xyz.shape[0]):
        xyz[i]=xyz[i]+offset
    pcdc.points=o3d.utility.Vector3dVector(xyz)
    
    return pcdc


def str2array(s):
    s=s[1:-2]
    # print(s)
    l=s.split()
    res=[eval(i) for i in l]
    return res
#%%

file=open(patht+'/params.txt','r')
lines=file.readlines()
file.close()
param=[]
for l in lines:
    r=l.split(':')
    # print(r[1])
    param.append(r[1])
node_num=int(param[0])
element_num=int(param[1])
A=str2array(param[2])
mean=str2array(param[3])
theta=float(param[4])
size=str2array(param[5])
# stime=float(param[6])
# dtime=etime-stime

pcd3=o3d.io.read_point_cloud(patht+'/interpcd_o.ply')
pcd4=o3d.io.read_point_cloud(patht+'/interpcd.ply')
pcds=o3d.io.read_point_cloud(patht+'/interpcd_s.ply')

file=open(patht+'/element_list.txt','r')
elementlines=file.readlines()
file.close()
element_list=[]
for l in elementlines[1:]:
    r=l.split(',')
    r=[int(i) for i in r[1:]]
    element_list.append(r)
    
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
#%%

# node_num=nx*ny

resfile=patht+'/test_v2.rpt'
with open(resfile,'r') as f:
    lines=f.readlines()
# res=[]
# for i in range(19,19+node_num):
#     pt=lines[i].split()[5:8]
#     pt=[float(n) for n in pt]
#     res.append(pt)
# res=np.array(res)

displacementfield=[]
# id=0.
for i in range(19,19+node_num):
    dis=lines[i].split()[1:]
    # del dis[1]
    dis=[float(n) for n in dis]
    # if dis[0]!=id:
    #     id=dis[0]
    displacementfield.append(dis)
# displacementfield.sort(key=lambda x:x[0])
df=np.array(displacementfield)

# xyz5=res
# # rgb4=rgb4
# pcd5=o3d.geometry.PointCloud()
# pcd5.points=o3d.utility.Vector3dVector(xyz5)
# pcd5.colors = o3d.utility.Vector3dVector(rgb4)
# o3d.visualization.draw_geometries([pcd5])

xyz4=np.asarray(pcd4.points)
rgb4=np.asarray(pcd4.colors)
xyz5=xyz4+df
pcd5=o3d.geometry.PointCloud()
pcd5.points=o3d.utility.Vector3dVector(xyz5)
pcd5.colors = o3d.utility.Vector3dVector(rgb4)
z=np.mean(xyz5,axis=0)[2]
#%%
#%%
# o3d.visualization.draw_geometries([pcd4,pcd5])
ny=size[1]
xyzd=xyz5[:ny]

xy=sum([p[0]*p[1] for p in xyzd])
xs=sum([p[0] for p in xyzd])
ys=sum([p[1] for p in xyzd])
x2s=sum([p[0]**2 for p in xyzd])

D=ny*xy-xs*ys
N=ny*x2s-xs**2
if N!=0:
    basedirection=D/N
else:
    basedirection=None

# pcd5=o3d.geometry.PointCloud()
# pcd5.points=o3d.utility.Vector3dVector(xyzd)
# pcd5.colors = o3d.utility.Vector3dVector(rgb4)
#%%
# o3d.visualization.draw_geometries([pcd4,pcd5])

#%% 0413
# xyznew,pcdnew,size=homomesh(pcd5,nx,basedirection)
# # o3d.visualization.draw_geometries([pcdnew,pcd5])
# st=time.time()
# count=1
# reverse_displacement_field=np.zeros((len(xyznew),3))
# lablist=np.zeros((len(xyznew),3))
# print(len(element_list))
# for e in element_list:
#     if count%100==0:
#         print(count)
#     count+=1
#     ps=[xyz4[pi-1] for pi in e]
#     psf=[xyz5[pi-1] for pi in e]
#     xs=[p[0] for p in psf]
#     ys=[p[1] for p in psf]
    
#     ind1=np.where((xyznew[:,0]>=min(xs))&(xyznew[:,0]<=max(xs)))
#     temp1=xyznew[ind1]
#     ind2=np.where((temp1[:,1]>=min(ys))&(temp1[:,1]<=max(ys)))
#     temp2=temp1[ind2]
#     ind=ind1[0][ind2]
#     #print(ind)
    
#     local_points=xyznew[ind]
#     element_displacement=[df[pi-1] for pi in e]
#     pl=[]
#     for i in ind:
#         p=xyznew[i]
#         pl.append(p)
#         intriangle,displacement=triangle_interpolation(p,psf,element_displacement)
#         if intriangle:
#             # lablist[i]=lab
#             reverse_displacement_field[i]=displacement
# et=time.time()
# print(et-st)
#%%
# xyznew,pcdnew,size=homomesh(pcd5,nx,basedirection)
# o3d.visualization.draw_geometries([pcdnew,pcd5])

xyznew=np.asarray(pcd3.points)
xyzs=np.asarray(pcds.points)

st=time.time()
count=1
reverse_displacement_field=np.zeros((len(xyznew),3))

reverse_displacement_field_s=np.zeros((len(xyzs),3))

lablist=np.zeros((len(xyznew),3))
print(len(element_list))

# 这里我实在是没法做测试，计算太久了，这部分是改成并行计算的代码，你测试一下吧，有用就留着，没用就删了，把下面注释的原始代码取消注释就行
def element_for(e):
    global count
    if count%100==0:
        print(count)
    count+=1
    ps=[xyz5[pi-1] for pi in e]
    psf=[xyz4[pi-1] for pi in e]
    xs=[p[0] for p in psf]
    ys=[p[1] for p in psf]
    
    ind1=np.where((xyznew[:,0]>=min(xs))&(xyznew[:,0]<=max(xs)))
    temp1=xyznew[ind1]
    ind2=np.where((temp1[:,1]>=min(ys))&(temp1[:,1]<=max(ys)))
    temp2=temp1[ind2]
    ind=ind1[0][ind2]
    #print(ind)
    local_points=xyznew[ind]
    element_displacement=[df[pi-1] for pi in e]
    pl=[]
    for i in ind:
        p=xyznew[i]
        pl.append(p)
        intriangle,displacement=triangle_interpolation(p,psf,element_displacement)
        if intriangle:
            # print('1')
            # lablist[i]=lab
            reverse_displacement_field[i]=displacement
            
    inds1=np.where((xyzs[:,0]>=min(xs))&(xyzs[:,0]<=max(xs)))
    temps1=xyzs[inds1]
    inds2=np.where((temps1[:,1]>=min(ys))&(temps1[:,1]<=max(ys)))
    temps2=temps1[inds2]
    inds=inds1[0][inds2]
    local_points_s=xyzs[inds]
    # element_displacement_s=[df[pi-1] for pi in e]
    pls=[]
    for i in inds:
        p=xyzs[i]
        pls.append(p)
        intriangle,displacement=triangle_interpolation(p,psf,element_displacement)
        if intriangle:
            # print('1')
            # lablist[i]=lab
            reverse_displacement_field_s[i]=displacement

element_pool = threadpool.ThreadPool(10)
requests = threadpool.makeRequests(element_for, element_list)
[element_pool.putRequest(req) for req in requests]
element_pool.wait()
# 删除的话，删到这里，取消注释下面的

# for e in element_list:
#     if count%100==0:
#         print(count)
#     count+=1
#     ps=[xyz5[pi-1] for pi in e]
#     psf=[xyz4[pi-1] for pi in e]
#     xs=[p[0] for p in psf]
#     ys=[p[1] for p in psf]
    
#     ind1=np.where((xyznew[:,0]>=min(xs))&(xyznew[:,0]<=max(xs)))
#     temp1=xyznew[ind1]
#     ind2=np.where((temp1[:,1]>=min(ys))&(temp1[:,1]<=max(ys)))
#     temp2=temp1[ind2]
#     ind=ind1[0][ind2]
#     #print(ind)
#     local_points=xyznew[ind]
#     element_displacement=[df[pi-1] for pi in e]
#     pl=[]
#     for i in ind:
#         p=xyznew[i]
#         pl.append(p)
#         intriangle,displacement=triangle_interpolation(p,psf,element_displacement)
#         if intriangle:
#             # print('1')
#             # lablist[i]=lab
#             reverse_displacement_field[i]=displacement
            
#     inds1=np.where((xyzs[:,0]>=min(xs))&(xyzs[:,0]<=max(xs)))
#     temps1=xyzs[inds1]
#     inds2=np.where((temps1[:,1]>=min(ys))&(temps1[:,1]<=max(ys)))
#     temps2=temps1[inds2]
#     inds=inds1[0][inds2]
#     local_points_s=xyzs[inds]
#     # element_displacement_s=[df[pi-1] for pi in e]
#     pls=[]
#     for i in inds:
#         p=xyzs[i]
#         pls.append(p)
#         intriangle,displacement=triangle_interpolation(p,psf,element_displacement)
#         if intriangle:
#             # print('1')
#             # lablist[i]=lab
#             reverse_displacement_field_s[i]=displacement
    
et=time.time()
print(et-st)
#%%
element_arr=[]
for m in range(size[0]-1):
    for n in range(size[1]-1):
        i=m*size[1]+n
        element_a1=[i,i+1,i+size[1]+1]
        element_a2=[i,i+size[1]+1,i+size[1]]
        element_arr.append(element_a1)
        element_arr.append(element_a2)

#%%
xyzr=np.array(xyznew)+reverse_displacement_field

ind=np.where(reverse_displacement_field!=[0,0,0])
xyzr=xyzr[ind[0]]
rgbr=np.asarray(pcd3.colors)[ind[0]]

xyzsr=np.array(xyzs)+reverse_displacement_field_s
#%%

pcdr=o3d.geometry.PointCloud()
pcdr.points=o3d.utility.Vector3dVector(xyzr)
pcdr.colors=o3d.utility.Vector3dVector(rgbr)
# pcdr.colors=o3d.utility.Vector3dVector(lablist)

pcdsr=o3d.geometry.PointCloud()
pcdsr.points=o3d.utility.Vector3dVector(xyzsr)
#%%
result_flatten=reverse_coordinate(pcdr,A,mean,theta)
result_flatten_mesh=o3d.geometry.TriangleMesh()
result_flatten_mesh.vertices=result_flatten.points
result_flatten_mesh.triangles=o3d.utility.Vector3iVector(np.asarray(element_arr))

result_r=reverse_coordinate(pcd3,A,mean,theta)
result_r_mesh=o3d.geometry.TriangleMesh()
result_r_mesh.vertices=result_r.points
result_r_mesh.triangles=o3d.utility.Vector3iVector(np.asarray(element_arr))

result_s=reverse_coordinate(pcdsr,A,mean,theta)
# result_s_mesh=o3d.geometry.TriangleMesh()
# result_s_mesh.vertices=result_s.points
# result_flatten_mesh.triangles=o3d.utility.Vector3iVector(np.asarray(element_arr))
#%%
o3d.io.write_triangle_mesh(path+'/result_flatten_mesh.ply',result_flatten_mesh)
o3d.io.write_triangle_mesh(path+'/result_r_mesh.ply',result_r_mesh)

o3d.io.write_point_cloud(path+'/result_r.ply',pcdr)
#%%
resolution=[800,600,3]
# xyz1=np.asarray(pcd1.points)
# rgb1=np.asarray(pcd1.colors)

xyz1=np.asarray(pcdr.points)
rgb1=np.asarray(pcdr.colors)

# xyzsr

margin=30
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
# cv2.imwrite(path+'/output.jpg',projectionimg)
# cv2.imwrite(path+'/gray.png',img_gray)

for i in range(len(xyz1)):
    p=xyz1[i]
    c=rgb1[i]*255
    c=[c[2],c[1],c[0]]
    pixelx=int((p[0]-minx)/dp)-1+margin
    pixely=int((maxy-p[1])/dp)-1+margin
    projectionimg[pixely][pixelx]=c
projectionimg=projectionimg.astype(np.uint8) 

img_gray = cv2.cvtColor(projectionimg,cv2.COLOR_BGR2GRAY)
maskindex=np.where(img_gray==0)
mask=np.zeros(resolution[:-1])
mask[maskindex]=255
mask=mask.astype(np.uint8)
dst= cv2.inpaint(projectionimg,mask,7,cv2.INPAINT_NS)
#%%
# masks=cv2.imread(r'G:\OS FEM Flatten\pack0801\test1208\m2\result\masks.jpg')
# masks=cv2.cvtColor(masks,cv2.COLOR_BGR2GRAY)
# ret,masks=cv2.threshold(masks,200,255,cv2.THRESH_BINARY)
#%%
masks=np.zeros(resolution[:-1])
for i in range(len(xyzsr)):
    p=xyzsr[i]
    c=255
    pixelx=int((p[0]-minx)/dp)-1+margin
    pixely=int((maxy-p[1])/dp)-1+margin
    masks[pixely][pixelx]=c
kernel=np.ones((6,6),np.uint8)
kernel1=cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
kernel2=cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
masks=cv2.dilate(masks,kernel1)
masks=cv2.erode(masks,kernel2)
masks=masks.astype(np.uint8)
dst2_ns= cv2.inpaint(dst,masks,10,cv2.INPAINT_NS)
dst2_tl= cv2.inpaint(dst,masks,10,cv2.INPAINT_TELEA)

# img1 = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
# img2=copy.deepcopy(img1)
# cimg=copy.deepcopy(dst)
# canny = cv2.Canny(img2, 100, 150)
# kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
# closed_canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE,kernel)

# lines = cv2.HoughLinesP(closed_canny,1,np.pi/72,30,minLineLength=5,maxLineGap=10)
# for i in range(len(lines)):
#     for x1, y1, x2, y2 in lines[i]:
#         cv2.line(closed_canny, (x1, y1), (x2, y2), 255, 2)
# projectionimg = cv2.morphologyEx(closed_canny, cv2.MORPH_CLOSE,kernel)
#%%
# cv2.imwrite(path+'/output.jpg',projectionimg)
# dst2=dst[margin:resolution[0]-margin,margin:resolution[1]-margin,:]
# dst=dst[]
cv2.imwrite(path+'/output.jpg',projectionimg)
cv2.imwrite(path+'/output2.jpg',dst)

cv2.imwrite(path+'/output3_ns.jpg',dst2_ns)
cv2.imwrite(path+'/output3_tl.jpg',dst2_tl)
cv2.imwrite(path+'/masks.jpg',masks)
#%%
cv2.imshow('masks',masks)
cv2.imshow('dst2_ns',dst2_ns)
cv2.imshow('dst2_tl',dst2_tl)
cv2.waitKey()
cv2.destroyAllWindows()
#%%
f=open(path+'/record.txt','w')
# f.write(str(dtime))
f.close()

print('post processing done')

# o3d.visualization.draw_geometries([pcd3],mesh_show_wireframe=False,mesh_show_back_face=True)










