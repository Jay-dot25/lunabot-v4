"""Layered, explainable rover traversability costs."""
from __future__ import annotations
import math
REASON={'OK':0,'UNKNOWN':1,'STALE':2,'INVALID_ELEVATION':3,'SEMANTIC_LETHAL':4,'SLOPE':5,'STEP':6,'INFLATED':7}
DEFAULT={'semantic_costs':[85,10,45,25,85,100,100,75,100],'slope_weight':1.0,'roughness_weight':25.0,'clearance_weight':20.0,'uncertainty_weight':20.0,'max_slope_deg':20.0,'max_cross_slope_deg':15.0,'max_step_m':0.25,'footprint_radius_m':0.45,'inflation_radius_m':0.25,'unknown_is_lethal':False,'stale_is_lethal':True}

def _finite(value):return isinstance(value,(int,float)) and math.isfinite(value)
def build_traversability(classes,confidence,elevation,stale,width,height,resolution,config=None):
 cfg={**DEFAULT,**(config or {})};size=width*height
 if any(len(layer)!=size for layer in (classes,confidence,elevation,stale)):raise ValueError('layer dimensions differ')
 lethal=[False]*size;reasons=[REASON['OK']]*size;slope=[math.nan]*size;roughness=[math.nan]*size
 for row in range(height):
  for col in range(width):
   i=row*width+col;label=int(classes[i])
   if label<0 or label>=len(cfg['semantic_costs']):label=0
   if label==0 and cfg['unknown_is_lethal']:lethal[i]=True;reasons[i]=REASON['UNKNOWN']
   if stale[i] and cfg['stale_is_lethal']:lethal[i]=True;reasons[i]=REASON['STALE']
   if not _finite(elevation[i]):lethal[i]=True;reasons[i]=REASON['INVALID_ELEVATION'];continue
   if cfg['semantic_costs'][label]>=100:lethal[i]=True;reasons[i]=REASON['SEMANTIC_LETHAL']
   neighbours=[];longitudinal=[];cross=[]
   for dr,dc in ((-1,0),(1,0),(0,-1),(0,1)):
    rr,cc=row+dr,col+dc
    if 0<=rr<height and 0<=cc<width and _finite(elevation[rr*width+cc]):
     delta=abs(elevation[rr*width+cc]-elevation[i]);neighbours.append(elevation[rr*width+cc]);(cross if dr else longitudinal).append(delta)
   if neighbours:
    maximum=max(abs(value-elevation[i]) for value in neighbours);long_slope=math.degrees(math.atan(max(longitudinal or [0])/resolution));cross_slope=math.degrees(math.atan(max(cross or [0])/resolution));slope[i]=max(long_slope,cross_slope);mean=sum(neighbours)/len(neighbours);roughness[i]=sum(abs(value-mean) for value in neighbours)/len(neighbours)
    if maximum>cfg['max_step_m']:lethal[i]=True;reasons[i]=REASON['STEP']
    elif long_slope>cfg['max_slope_deg'] or cross_slope>cfg['max_cross_slope_deg']:lethal[i]=True;reasons[i]=REASON['SLOPE']
 radius=int(math.ceil((cfg['footprint_radius_m']+cfg['inflation_radius_m'])/resolution));inflated=lethal[:]
 for row in range(height):
  for col in range(width):
   if not lethal[row*width+col]:continue
   for dr in range(-radius,radius+1):
    for dc in range(-radius,radius+1):
     rr,cc=row+dr,col+dc
     if 0<=rr<height and 0<=cc<width and math.hypot(dr,dc)*resolution<=cfg['footprint_radius_m']+cfg['inflation_radius_m']:
      j=rr*width+cc
      if not inflated[j]:inflated[j]=True;reasons[j]=REASON['INFLATED']
 costs=[]
 for i,label in enumerate(classes):
  label=int(label) if 0<=int(label)<len(cfg['semantic_costs']) else 0
  if inflated[i]:costs.append(100);continue
  slope_cost=0 if not _finite(slope[i]) else min(100,slope[i]/cfg['max_slope_deg']*100)
  rough_cost=0 if not _finite(roughness[i]) else min(100,roughness[i]/max(cfg['max_step_m'],1e-6)*100)
  uncertainty=1-max(0.,min(1.,float(confidence[i])))
  clearance=0
  hazards=[j for j,value in enumerate(lethal) if value]
  if hazards:
   row,col=divmod(i,width);distance=min(math.hypot(row-divmod(j,width)[0],col-divmod(j,width)[1])*resolution for j in hazards);clearance=max(0.,1-distance/max(cfg['inflation_radius_m']+cfg['footprint_radius_m'],resolution))*100
  total=cfg['semantic_costs'][label]+cfg['slope_weight']*slope_cost+cfg['roughness_weight']*rough_cost/100+cfg['clearance_weight']*clearance/100+cfg['uncertainty_weight']*uncertainty
  costs.append(max(0,min(99,round(total))))
 return {'cost':costs,'reason':reasons,'slope_deg':slope,'roughness':roughness,'lethal':inflated}
