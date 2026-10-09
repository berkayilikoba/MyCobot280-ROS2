import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
b,out=sys.argv[1],sys.argv[2]
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
T=[];P=[]
while r.has_next():
    _,d,t=r.read_next(); m=deserialize_message(d,LaserScan)
    rr=np.array(m.ranges); a=m.angle_min+np.arange(len(rr))*m.angle_increment
    k=np.isfinite(rr)&(rr>0.25)&(rr<8)
    T.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9); P.append(np.c_[rr[k]*np.cos(a[k]),rr[k]*np.sin(a[k])].astype(np.float32))
np.save(out+'_scanT.npy',np.array(T)); np.save(out+'_scanP.npy',np.array(P,dtype=object),allow_pickle=True)
print(len(T))
