#!/usr/bin/env python3
"""Train Phase E MLP only from annotated, captured Gazebo RGB-D frames."""
import argparse, json
from pathlib import Path
import numpy as np
LABELS=['BEDROCK','REGOLITH','ROCK','CRATER','SHADOW']
FEATURES=['red','green','blue','depth_norm','row_norm','depth_gradient','texture']

def read_pgm(path, shape):
    data=path.read_bytes(); parts=data.split(b'\n',3)
    if len(parts)!=4 or parts[0]!=b'P5': raise ValueError(f'bad mask {path}')
    a=np.frombuffer(parts[3],np.uint8)
    if a.size!=shape[0]*shape[1]: raise ValueError(f'shape mismatch {path}')
    return a.reshape(shape)

def features(rgb, depth, max_depth=12.):
    rgb=rgb.astype(np.float32)/255.; h,w=rgb.shape[:2]
    valid=np.isfinite(depth)&(depth>.05)&(depth<=max_depth)
    safe=np.where(valid,depth,max_depth); gy,gx=np.gradient(safe)
    grad=np.clip(np.hypot(gx,gy)/max(.25,max_depth*.08),0,1)
    lum=rgb.mean(2); tx=np.zeros_like(lum); ty=np.zeros_like(lum)
    tx[:,1:]=np.abs(lum[:,1:]-lum[:,:-1]); ty[1:]=np.abs(lum[1:]-lum[:-1])
    texture=np.clip((tx+ty)*4,0,1); rows=np.broadcast_to(np.linspace(0,1,h)[:,None],(h,w))
    return np.dstack((rgb,np.clip(safe/max_depth,0,1),rows,grad,texture)).astype(np.float32),valid

def load(directory, per_class, seed):
    rng=np.random.default_rng(seed); frames=[]; support=np.zeros(5,dtype=np.int64)
    for archive in sorted(directory.glob('frame_*.npz')):
        mask_path=archive.with_suffix('.mask.pgm')
        if not mask_path.exists(): continue
        z=np.load(archive); rgb=z['rgb']; depth=z['depth']; mask=read_pgm(mask_path,depth.shape)
        f,valid=features(rgb,depth); mask=np.where(valid,mask,255)
        support += np.array([(mask==i).sum() for i in range(5)])
        frames.append((archive.name,f,mask))
    if len(frames)<10: raise SystemExit('need at least 10 annotated capture frames')
    missing=[LABELS[i] for i,n in enumerate(support) if n<500]
    if missing: raise SystemExit('need >=500 valid labeled pixels per class: '+','.join(missing))
    rng.shuffle(frames); cut=max(1,int(len(frames)*.8)); groups=[]
    for subset in (frames[:cut],frames[cut:]):
        xs=[]; ys=[]
        for _,f,m in subset:
            for lab in range(5):
                idx=np.flatnonzero(m.ravel()==lab)
                take=rng.choice(idx,min(per_class,len(idx)),replace=False)
                xs.append(f.reshape(-1,7)[take]); ys.append(np.full(len(take),lab))
        groups.append((np.concatenate(xs),np.concatenate(ys)))
    return groups,frames,support

def train(x,y,seed,epochs):
    rng=np.random.default_rng(seed); dims=(7,12,8,5)
    w1=rng.normal(0,np.sqrt(2/7),(7,12)).astype('f4'); b1=np.zeros(12,'f4')
    w2=rng.normal(0,np.sqrt(2/12),(12,8)).astype('f4'); b2=np.zeros(8,'f4')
    w3=rng.normal(0,np.sqrt(2/8),(8,5)).astype('f4'); b3=np.zeros(5,'f4')
    for _ in range(epochs):
        for batch in np.array_split(rng.permutation(len(x)),max(1,len(x)//256)):
            xb=x[batch]; yb=y[batch]; h1=np.maximum(0,xb@w1+b1); h2=np.maximum(0,h1@w2+b2)
            logits=h2@w3+b3; ex=np.exp(logits-logits.max(1,keepdims=True)); d=ex/ex.sum(1,keepdims=True)
            d[np.arange(len(batch)),yb]-=1; d/=len(batch); dw3=h2.T@d; db3=d.sum(0)
            d2=(d@w3.T)*(h2>0); dw2=h1.T@d2; db2=d2.sum(0)
            d1=(d2@w2.T)*(h1>0); dw1=xb.T@d1; db1=d1.sum(0)
            for p,g in ((w1,dw1),(b1,db1),(w2,dw2),(b2,db2),(w3,dw3),(b3,db3)): p-=.01*g
    return [w1,b1,w2,b2,w3,b3]
def predict(x,p): return np.argmax(np.maximum(0,np.maximum(0,x@p[0]+p[1])@p[2]+p[3])@p[4]+p[5],1)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('dataset'); ap.add_argument('--output',default='models/terrain_mlp_v2.json'); ap.add_argument('--seed',type=int,default=2504); ap.add_argument('--epochs',type=int,default=30); ap.add_argument('--pixels-per-class-frame',type=int,default=1500); a=ap.parse_args()
    directory=Path(a.dataset).resolve(); (trainset,testset),frames,support=load(directory,a.pixels_per_class_frame,a.seed)
    p=train(*trainset,a.seed,a.epochs); pred=predict(testset[0],p); truth=testset[1]
    cm=np.zeros((5,5),dtype=int)
    for t,q in zip(truth,pred): cm[t,q]+=1
    recall=np.diag(cm)/np.maximum(1,cm.sum(1)); precision=np.diag(cm)/np.maximum(1,cm.sum(0)); accuracy=float((pred==truth).mean())
    if np.any(recall<.55): raise SystemExit(f'held-out per-class recall below 0.55: {recall}')
    artifact={'format':'lunabot_mlp_v2','architecture':[7,12,8,5],'activation':'relu','labels':LABELS,'features':FEATURES,'training':{'method':'supervised annotated Gazebo RGB-D captures','dataset':directory.name,'frame_count':len(frames),'split':'whole-frame 80/20','seed':a.seed,'epochs':a.epochs,'pixel_support':support.tolist(),'held_out_accuracy':accuracy,'per_class_precision':precision.tolist(),'per_class_recall':recall.tolist(),'confusion_matrix':cm.tolist()},'w1':p[0].tolist(),'b1':p[1].tolist(),'w2':p[2].tolist(),'b2':p[3].tolist(),'w3':p[4].tolist(),'b3':p[5].tolist()}
    Path(a.output).write_text(json.dumps(artifact,indent=2)+'\n'); print(f'wrote {a.output} accuracy={accuracy:.3%} recall={recall}')
if __name__=='__main__': main()
