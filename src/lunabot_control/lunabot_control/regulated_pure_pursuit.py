"""Deterministic regulated pure-pursuit controller core."""
from dataclasses import dataclass
import math

def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))
def clamp(value,low,high):return max(low,min(high,value))
@dataclass(frozen=True)
class Command:
 linear:float;angular:float;cross_track_error:float;heading_error:float;curvature:float;target_index:int
class RegulatedPurePursuit:
 def __init__(self,lookahead=0.8,max_linear=.3,max_angular=.8,max_accel=.25,max_decel=.5,curvature_gain=1.,goal_tolerance=.2):
  self.lookahead=max(.05,lookahead);self.max_linear=max(0.,max_linear);self.max_angular=max(0.,max_angular);self.max_accel=max(0.,max_accel);self.max_decel=max(0.,max_decel);self.curvature_gain=max(0.,curvature_gain);self.goal_tolerance=max(.01,goal_tolerance);self.last_linear=0.
 def reset(self):self.last_linear=0.
 def command(self,path,pose,dt,clearance=1.,roughness=0.,uncertainty=0.):
  if not path:return Command(0.,0.,0.,0.,0.,-1)
  x,y,yaw=pose;nearest=min(range(len(path)),key=lambda i:(path[i][0]-x)**2+(path[i][1]-y)**2)
  cross=math.hypot(path[nearest][0]-x,path[nearest][1]-y)
  target=nearest;distance=0.
  while target+1<len(path) and distance<self.lookahead:
   distance+=math.hypot(path[target+1][0]-path[target][0],path[target+1][1]-path[target][1]);target+=1
  gx,gy=path[-1]
  if math.hypot(gx-x,gy-y)<=self.goal_tolerance:self.last_linear=0.;return Command(0.,0.,cross,0.,0.,target)
  tx,ty=path[target];heading=wrap(math.atan2(ty-y,tx-x)-yaw);look=max(.05,math.hypot(tx-x,ty-y));curvature=2.*math.sin(heading)/look
  curvature_factor=1./(1.+self.curvature_gain*abs(curvature));clearance_factor=clamp(clearance/max(self.lookahead,.05),.1,1.);terrain_factor=clamp(1.-.7*roughness-.7*uncertainty,.1,1.)
  desired=self.max_linear*min(curvature_factor,clearance_factor,terrain_factor)
  dt=max(0.,dt);desired=clamp(desired,self.last_linear-self.max_decel*dt,self.last_linear+self.max_accel*dt);angular=clamp(desired*curvature,-self.max_angular,self.max_angular);self.last_linear=desired
  return Command(desired,angular,cross,heading,curvature,target)
