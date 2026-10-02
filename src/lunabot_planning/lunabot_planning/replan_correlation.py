"""Explicit obstacle-to-map-to-D*-Lite replan event correlation."""
class ReplanCorrelation:
 def __init__(self):self.reset()
 def reset(self):
  self.initial_path=set();self.blocked=set();self.detection_ns=None;self.map_ns=None;self.request_ns=None;self.finish_ns=None;self.new_path=set();self.collisions=0;self.goal_reached=False
 def set_initial_path(self,cells):self.initial_path=set(cells)
 def sensor_detection(self,stamp_ns,cells):
  cells=set(cells)
  if not cells & self.initial_path:return False
  self.detection_ns=int(stamp_ns);self.blocked=cells & self.initial_path;return True
 def lethal_update(self,stamp_ns,cells):
  cells=set(cells)
  if self.detection_ns is None or not cells & self.blocked:return False
  self.map_ns=int(stamp_ns);return self.map_ns>=self.detection_ns
 def replan_requested(self,stamp_ns,reason):
  if self.map_ns is None or reason!='obstacle':return False
  self.request_ns=int(stamp_ns);return self.request_ns>=self.map_ns
 def path_published(self,stamp_ns,cells):
  if self.request_ns is None:return False
  self.finish_ns=int(stamp_ns);self.new_path=set(cells);return self.finish_ns>=self.request_ns
 def result(self):
  valid=(self.detection_ns is not None and self.map_ns is not None and self.request_ns is not None and self.finish_ns is not None and bool(self.blocked) and not bool(self.new_path & self.blocked))
  return {'valid_replan':valid,'mission_success':valid and self.goal_reached and self.collisions==0,'detection_to_map_ms':(self.map_ns-self.detection_ns)/1e6 if self.map_ns is not None else None,'replan_time_ms':(self.finish_ns-self.request_ns)/1e6 if self.finish_ns is not None else None,'total_latency_ms':(self.finish_ns-self.detection_ns)/1e6 if self.finish_ns is not None else None,'blocked_cells':len(self.blocked)}
