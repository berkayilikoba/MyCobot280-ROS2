import sys, math, numpy as np
sys.argv=[sys.argv[0]]+sys.argv[1:]
exec(open(sys.argv[0].replace('icp_ardisik','icp_fonk')).read())
b=sys.argv[1]; LX=0.1; STEP=int(sys.argv[2]) if len(sys.argv)>2 else 2
O=np.load(b+'_O.npy'); I=np.load(b+'_I.npy'); T=np.load(b+'_scanT.npy'); P=np.load(b+'_scanP.npy',allow_pickle=True)
oy=np.unwrap(O[:,4]); ot=O[:,1]; it=I[:,1]; iy=np.unwrap(I[:,3])
def odom_at(t):
    j=np.searchsorted(ot,t)
    if j<=0 or j>=len(ot) or ot[j]-ot[j-1]>0.1: return None
    f=(t-ot[j-1])/(ot[j]-ot[j-1]); return [O[j-1,2]+f*(O[j,2]-O[j-1,2]),O[j-1,3]+f*(O[j,3]-O[j-1,3]),oy[j-1]+f*(oy[j]-oy[j-1])]
rows=[]
for a in range(0,len(T)-STEP):
    c=a+STEP; pa,pc=odom_at(T[a]),odom_at(T[c])
    if pa is None or pc is None or T[c]-T[a]>0.6: continue
    dth=pc[2]-pa[2]; ca,sa=math.cos(pa[2]),math.sin(pa[2])
    dx=ca*(pc[0]-pa[0])+sa*(pc[1]-pa[1]); dy=-sa*(pc[0]-pa[0])+ca*(pc[1]-pa[1])
    if abs(dth)<1e-4 and math.hypot(dx,dy)<1e-4: continue          # durağan çiftleri atla
    res=icp(P[a].astype(float),P[c].astype(float),dx+LX*math.cos(dth)-LX,dy+LX*math.sin(dth),dth)
    if res is None or res[4]<0.7: continue
    x,y,th=res[:3]; dt=T[c]-T[a]
    gyro=np.interp(T[c],it,iy)-np.interp(T[a],it,iy)   # IMU yönelim artışı
    rows.append((T[a],dt,dth,th,gyro,math.hypot(dx,dy),math.hypot(x-(LX*math.cos(th)-LX),y-LX*math.sin(th))))
R=np.array(rows); np.save(b+'_icpR.npy',R)
w=np.abs(R[:,3])/R[:,1]
print(f'{b}: cift {len(R)}')
print(' |w_icp| aralik    n   sum(odom_dyaw)/sum(icp)  sum(imu)/sum(icp)')
for lo,hi in [(0.0,0.05),(0.05,0.1),(0.1,0.2),(0.2,0.4),(0.4,0.8),(0.8,3)]:
    m=(w>=lo)&(w<hi)
    if m.sum()<5: continue
    s=np.sign(R[m,3]); print(f'  {lo:4.2f}-{hi:4.2f}  {m.sum():5d}   {np.sum(R[m,2]*s)/np.sum(np.abs(R[m,3])):6.3f}   {np.sum(R[m,4]*s)/np.sum(np.abs(R[m,3])):6.3f}')
mv=(R[:,6]>0.03)&(w<0.1)
print(f' duz hareket ({mv.sum()} cift): odom/icp mesafe {R[mv,5].sum()/R[mv,6].sum():.3f}')
