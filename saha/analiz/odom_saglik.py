#!/usr/bin/env python3
"""Odom/IMU sağlığı: hız, gecikme, boşluklar, odom-IMU yön farkı, AMCL map->odom sıçramaları.
Kullanım: odom_saglik.py <bag> <çıktı_öneki> [odom_konusu=/odom]  -> <önek>_O.npy, _I.npy, (_M.npy)"""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
b=sys.argv[1]
ODOM=sys.argv[3] if len(sys.argv)>3 else '/odom'   # ör. /odom_duz
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=[ODOM,'/tf','/imu']))
O=[];M=[];I=[]
while r.has_next():
    tp,d,t=r.read_next(); t*=1e-9
    if tp==ODOM:
        m=deserialize_message(d,Odometry); p=m.pose.pose
        O.append((t,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.position.x,p.position.y,yaw(p.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z))
    elif tp=='/imu':
        m=deserialize_message(d,Imu); I.append((t,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.angular_velocity.z,yaw(m.orientation)))
    else:
        for tr in deserialize_message(d,TFMessage).transforms:
            if tr.header.frame_id=='map' and tr.child_frame_id=='odom':
                q=tr.transform.rotation; M.append((t,tr.transform.translation.x,tr.transform.translation.y,yaw(q)))
O=np.array(O); I=np.array(I); M=np.array(M)
dt=np.diff(O[:,0]); print(f'odom n={len(O)} hiz={1/np.median(dt):.1f}Hz, >0.2s bosluk={np.sum(dt>0.2)}, en buyuk={dt.max():.2f}s')
lat=O[:,0]-O[:,1]; print(f'odom alis-damga: medyan {np.median(lat):.3f}s p95 {np.percentile(lat,95):.3f} max {lat.max():.2f} min {lat.min():.3f}')
ds=np.diff(O[:,1]); print(f'odom damga geri gitme={np.sum(ds<=0)}')
lat=I[:,0]-I[:,1]; print(f'imu n={len(I)} hiz={1/np.median(np.diff(I[:,0])):.1f}Hz alis-damga medyan {np.median(lat):.3f} p95 {np.percentile(lat,95):.3f}')
# odom pozu ile twist tutarlılığı
dtt=np.diff(O[:,1]); ok=dtt>1e-3
vx_p=(np.hypot(np.diff(O[:,2]),np.diff(O[:,3]))/np.where(ok,dtt,1))[ok]; vx_t=np.abs(O[1:,5])[ok]
mv=vx_t>0.05; print(f'poz turevi/twist v orani (hareketli) medyan {np.median(vx_p[mv]/vx_t[mv]):.3f}')
dist=np.sum(np.hypot(np.diff(O[:,2]),np.diff(O[:,3]))); rot=np.sum(np.abs(np.diff(np.unwrap(O[:,4])))); print(f'yol {dist:.1f} m, toplam donus {math.degrees(rot):.0f} deg')
# odom yaw vs imu yaw
iy=np.interp(O[:,1],I[:,1],np.unwrap(I[:,3])); oy=np.unwrap(O[:,4]); e=(oy-oy[0])-(iy-iy[0]); print(f'odom yaw - imu yaw kayma: son {math.degrees(e[-1]):.2f} deg, maks |{math.degrees(np.abs(e).max()):.2f}|')
if len(M):
    print(f'map->odom n={len(M)}: x {M[0,1]:.2f}->{M[-1,1]:.2f}, y {M[0,2]:.2f}->{M[-1,2]:.2f}, yaw {math.degrees(M[0,3]):.1f}->{math.degrees(M[-1,3]):.1f} deg')
    j=np.hypot(np.diff(M[:,1]),np.diff(M[:,2])); jy=np.abs(np.diff(np.unwrap(M[:,3])))
    print(f'  map->odom sicramalari: >5cm {np.sum(j>0.05)}, >10cm {np.sum(j>0.1)}, maks {j.max():.2f} m; yaw >2deg {np.sum(jy>math.radians(2))}, maks {math.degrees(jy.max()):.1f} deg')
    np.save(sys.argv[2]+'_M.npy',M)
np.save(sys.argv[2]+'_O.npy',O); np.save(sys.argv[2]+'_I.npy',I)
