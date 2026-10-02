"""Calibrated pinhole projection and temporal semantic evidence fusion."""
from __future__ import annotations
import math

HAZARDS={5,6,8}

def project_pixel(u,v,depth,intrinsics,transform):
    """Project a depth pixel through camera intrinsics and a 4x4 row-major transform."""
    if not math.isfinite(depth) or depth<=0: raise ValueError('depth must be positive and finite')
    fx,fy,cx,cy=(float(intrinsics[k]) for k in ('fx','fy','cx','cy'))
    if fx<=0 or fy<=0: raise ValueError('focal lengths must be positive')
    camera=((u-cx)*depth/fx,(v-cy)*depth/fy,depth,1.0)
    if len(transform)!=16: raise ValueError('transform must contain 16 values')
    return tuple(sum(float(transform[row*4+column])*camera[column] for column in range(4)) for row in range(3))

class SemanticFusionGrid:
    """Bounded rolling sparse grid using Dirichlet-style accumulated evidence."""
    def __init__(self,width=200,height=200,resolution=.1,origin=(-10.,-10.),classes=9,stale_after_s=5.):
        if width<=0 or height<=0 or resolution<=0: raise ValueError('invalid grid geometry')
        self.width,self.height,self.resolution,self.origin,self.classes=width,height,float(resolution),origin,classes
        self.stale_after_ns=int(stale_after_s*1e9);self.cells={}
    def index(self,x,y):
        col=int(math.floor((x-self.origin[0])/self.resolution));row=int(math.floor((y-self.origin[1])/self.resolution))
        return (row,col) if 0<=row<self.height and 0<=col<self.width else None
    def observe(self,x,y,z,class_id,confidence,stamp_ns):
        if class_id<=0 or class_id>=self.classes or not math.isfinite(confidence) or confidence<=0:return False
        key=self.index(x,y)
        if key is None:return False
        cell=self.cells.setdefault(key,{'evidence':[0.25]*self.classes,'elevation_sum':0.,'weight':0.,'last_observed_ns':0})
        weight=min(1.,float(confidence))*(3.0 if class_id in HAZARDS else 1.0)
        cell['evidence'][class_id]+=weight;cell['elevation_sum']+=float(z)*weight;cell['weight']+=weight;cell['last_observed_ns']=max(cell['last_observed_ns'],int(stamp_ns));return True
    def value(self,row,col,now_ns):
        cell=self.cells.get((row,col))
        if not cell:return {'class_id':0,'confidence':0.,'elevation':math.nan,'last_observed_ns':0,'stale':True}
        evidence=cell['evidence'];total=sum(evidence);hazard=max(HAZARDS,key=lambda index:evidence[index]);best=max(range(1,self.classes),key=lambda index:evidence[index])
        # Once meaningful hazard evidence exists, traversable observations cannot erase it.
        if evidence[hazard]>=1.0:best=hazard
        stale=int(now_ns)-cell['last_observed_ns']>self.stale_after_ns
        return {'class_id':0 if stale else best,'confidence':0. if stale else evidence[best]/total,'elevation':cell['elevation_sum']/cell['weight'] if cell['weight'] else math.nan,'last_observed_ns':cell['last_observed_ns'],'stale':stale}
    def dense_layers(self,now_ns):
        layers={name:[] for name in ('semantic_class','semantic_confidence','elevation','last_observed')}
        for row in range(self.height):
            for col in range(self.width):
                value=self.value(row,col,now_ns);layers['semantic_class'].append(value['class_id']);layers['semantic_confidence'].append(value['confidence']);layers['elevation'].append(value['elevation']);layers['last_observed'].append(value['last_observed_ns'])
        return layers
