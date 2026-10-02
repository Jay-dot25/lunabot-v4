"""Fail-closed command arbitration and safety decision core."""
from dataclasses import dataclass
import math
@dataclass(frozen=True)
class Decision:
 linear:float;angular:float;safe:bool;state:str;reason:str;source:str
class SafetySupervisor:
 def __init__(self,max_linear=.35,max_angular=1.,command_timeout=.3,localization_timeout=.5,sensor_timeout=.5,path_timeout=1.,max_slope=25.):
  self.max_linear=max_linear;self.max_angular=max_angular;self.command_timeout=command_timeout;self.localization_timeout=localization_timeout;self.sensor_timeout=sensor_timeout;self.path_timeout=path_timeout;self.max_slope=max_slope;self.estop=False
 def set_estop(self,enabled):self.estop=bool(enabled)
 def decide(self,now,nav=None,teleop=None,localization_stamp=None,sensor_stamp=None,path_stamp=None,slope=0.,obstacle=False):
  stop=lambda reason,state='stopped':Decision(0.,0.,False,state,reason,'none')
  if self.estop:return stop('emergency_stop','estop')
  if localization_stamp is None or now-localization_stamp>self.localization_timeout:return stop('localization_lost')
  if sensor_stamp is None or now-sensor_stamp>self.sensor_timeout:return stop('sensor_timeout')
  if path_stamp is None or now-path_stamp>self.path_timeout:return stop('path_expired')
  if not math.isfinite(slope) or abs(slope)>self.max_slope:return stop('excessive_slope')
  if obstacle:return stop('obstacle')
  candidates=[]
  if nav and now-nav[0]<=self.command_timeout:candidates.append(('navigation',nav))
  if teleop and now-teleop[0]<=self.command_timeout:candidates.append(('teleop',teleop))
  if not candidates:return stop('command_timeout')
  source,(_,linear,angular)=candidates[-1]
  if not math.isfinite(linear) or not math.isfinite(angular):return stop('invalid_command','fault')
  return Decision(max(-self.max_linear,min(self.max_linear,linear)),max(-self.max_angular,min(self.max_angular,angular)),True,'safe','none',source)
