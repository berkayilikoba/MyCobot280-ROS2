import math, numpy as np
from scipy.spatial import cKDTree
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
