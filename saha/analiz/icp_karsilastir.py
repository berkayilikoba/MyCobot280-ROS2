import sys, math, numpy as np
from scipy.spatial import cKDTree
b=sys.argv[1]
O=np.load(b+'_O.npy'); T=np.load(b+'_scanT.npy'); P=np.load(b+'_scanP.npy',allow_pickle=True)
LX=float(sys.argv[2]) if len(sys.argv)>2 else 0.0   # lazerin base_link'e göre x ofseti
def icp(A,B,x0,y0,th0,it=40):
    """B'yi (sonraki tarama) A çerçevesine taşıyan dönüşüm; başlangıç odomdan."""
    tree=cKDTree(A); x,y,th=x0,y0,th0
    for _ in range(it):
        c,s=math.cos(th),math.sin(th); Bt=B@np.array([[c,s],[-s,c]])+[x,y]
        d,i=tree.query(Bt); k=d<np.percentile(d,75); k&=d<0.3
        if k.sum()<30: return None
        p=Bt[k]; q=A[i[k]]; mp,mq=p.mean(0),q.mean(0)
        H=(p-mp).T@(q-mq); U,_,Vt=np.linalg.svd(H); R=Vt.T@U.T
        if np.linalg.det(R)<0: return None
        dth=math.atan2(R[1,0],R[0,0]); t=mq-R@mp
        # birleştir
        c2,s2=math.cos(dth),math.sin(dth); x,y=c2*x-s2*y+t[0],s2*x+c2*y+t[1]; th+=dth
        if abs(dth)<1e-5 and np.hypot(*t)<1e-5: break
    c,s=math.cos(th),math.sin(th); Bt=B@np.array([[c,s],[-s,c]])+[x,y]; d,_=tree.query(Bt)
    return x,y,th,float(np.median(d)),float(np.mean(d<0.05))
# odomu tarama zamanlarına ara değerle
oy=np.unwrap(O[:,4]); ot=O[:,1]
def odom_at(t):
    j=np.searchsorted(ot,t); 
    if j<=0 or j>=len(ot) or ot[j]-ot[j-1]>0.15: return None
    f=(t-ot[j-1])/(ot[j]-ot[j-1]); return [O[j-1,2]+f*(O[j,2]-O[j-1,2]),O[j-1,3]+f*(O[j,3]-O[j-1,3]),oy[j-1]+f*(oy[j]-oy[j-1])]
# durağan taramalar: odom hızı ±0.3 s içinde ~0
still=[]
for k,t in enumerate(T):
    m=(ot>t-0.4)&(ot<t+0.4)
    if m.sum()>10 and np.all(np.abs(O[m,5])<0.005) and np.all(np.abs(O[m,6])<0.01): still.append(k)
# durağan blokları grupla, her bloktan bir tarama
blk=[]; 
for k in still:
    if blk and k-blk[-1][-1]<=2: blk[-1].append(k)
    else: blk.append([k])
reps=[bl[len(bl)//2] for bl in blk if len(bl)>=3]
print('durağan blok',len(reps))
rows=[]
for a,c in zip(reps[:-1],reps[1:]):
    pa,pc=odom_at(T[a]),odom_at(T[c])
    if pa is None or pc is None: continue
    dth=pc[2]-pa[2]; ca,sa=math.cos(pa[2]),math.sin(pa[2])
    dx=ca*(pc[0]-pa[0])+sa*(pc[1]-pa[1]); dy=-sa*(pc[0]-pa[0])+ca*(pc[1]-pa[1])
    # base->laser: lazer çerçevesinde başlangıç tahmini (yalnız x ofseti)
    lx=dx+LX*math.cos(dth)-LX; ly=dy+LX*math.sin(dth)
    res=icp(P[a].astype(float),P[c].astype(float),lx,ly,dth)
    if res is None: continue
    x,y,th,med,fit=res
    if fit<0.6: continue
    rows.append((T[a]-T[0],T[c]-T[0],math.degrees(dth),math.degrees(th),math.hypot(dx,dy),math.hypot(x-(LX*math.cos(th)-LX),y-LX*math.sin(th)),fit))
R=np.array(rows)
np.set_printoptions(suppress=True,linewidth=150)
print('t0 t1 odom_dyaw icp_dyaw odom_d icp_d fit'); print(np.round(R,3))
rot=np.abs(R[:,3])>20; 
if rot.any(): print('DONUS: odom/icp yaw orani', np.round(R[rot,2]/R[rot,3],3), 'toplam', round(R[rot,2].sum()/R[rot,3].sum(),4))
tr=(R[:,5]>0.3)&(np.abs(R[:,3])<15)
if tr.any(): print('DUZ: odom/icp mesafe', np.round(R[tr,4]/R[tr,5],3), 'toplam', round(R[tr,4].sum()/R[tr,5].sum(),4))
