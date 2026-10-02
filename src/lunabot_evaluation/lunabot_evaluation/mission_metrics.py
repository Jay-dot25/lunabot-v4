"""Quantitative mission and physics-contact metric accumulator."""
import math
class MissionMetricsCollector:
 def __init__(self,terrain_classes=9):
  self.start_time=None;self.end_time=None;self.last_pose=None;self.executed=0.;self.planned=0.;self.cost=0.;self.minimum_clearance=math.inf;self.replan_times=[];self.collisions=[];self.active_contacts={};self.cross_track=[];self.terrain_distance=[0.]*terrain_classes;self.failure_reason='';self.success=False
 def start(self,stamp):self.start_time=float(stamp)
 def set_plan(self,points):self.planned=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))
 def pose(self,stamp,x,y,terrain_class=None,traversability_cost=0.):
  if self.last_pose:
   distance=math.hypot(x-self.last_pose[1],y-self.last_pose[2]);self.executed+=distance;self.cost+=distance*traversability_cost
   if terrain_class is not None and 0<=terrain_class<len(self.terrain_distance):self.terrain_distance[terrain_class]+=distance
  self.last_pose=(stamp,x,y)
 def clearance(self,value):
  if math.isfinite(value):self.minimum_clearance=min(self.minimum_clearance,value)
 def replan(self,milliseconds):self.replan_times.append(float(milliseconds))
 def tracking_error(self,value):
  if math.isfinite(value):self.cross_track.append(float(value))
 def contact(self,stamp,object_name,active,relative_speed=0.,impulse=0.):
  if active and object_name not in self.active_contacts:self.active_contacts[object_name]=(stamp,relative_speed,impulse)
  elif not active and object_name in self.active_contacts:
   begin,speed,force=self.active_contacts.pop(object_name);self.collisions.append({'object':object_name,'start':begin,'end':stamp,'duration':max(0.,stamp-begin),'relative_speed':speed,'impulse':force})
 def finish(self,stamp,success,failure_reason=''):
  for name in list(self.active_contacts):self.contact(stamp,name,False)
  self.end_time=float(stamp);self.success=bool(success);self.failure_reason=failure_reason
 def result(self):
  duration=max(0.,(self.end_time-self.start_time)) if self.start_time is not None and self.end_time is not None else 0.;rmse=math.sqrt(sum(x*x for x in self.cross_track)/len(self.cross_track)) if self.cross_track else 0.
  return {'complete':self.end_time is not None,'success':self.success,'failure_reason':self.failure_reason,'planned_path_length_m':self.planned,'executed_path_length_m':self.executed,'mission_duration_s':duration,'path_efficiency':self.planned/self.executed if self.executed else 0.,'accumulated_traversability_cost':self.cost,'minimum_clearance_m':self.minimum_clearance if math.isfinite(self.minimum_clearance) else -1.,'replan_count':len(self.replan_times),'mean_replan_time_ms':sum(self.replan_times)/len(self.replan_times) if self.replan_times else 0.,'max_replan_time_ms':max(self.replan_times,default=0.),'collision_count':len(self.collisions),'collision_duration_s':sum((float(c['duration']) for c in self.collisions),0.0),'controller_cross_track_rmse_m':rmse,'terrain_distance_m':self.terrain_distance,'collisions':self.collisions}
