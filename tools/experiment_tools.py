"""Deterministic experiment matrix and statistical summary helpers."""
import itertools,json,math
from pathlib import Path

def load_manifest(path):
 data=json.loads(Path(path).read_text());factors=data['factors'];names=sorted(factors);trials=[]
 for values in itertools.product(*(factors[n] for n in names)):
  base=dict(zip(names,values))
  for repeat in range(int(data.get('repetitions',1))):trials.append({**base,'repeat':repeat,'trial_id':f"trial-{len(trials):05d}"})
 return data,trials
def mean_ci(values):
 values=[float(x) for x in values]
 if not values:return {'n':0,'mean':None,'ci95_low':None,'ci95_high':None}
 mean=sum(values)/len(values)
 if len(values)==1:return {'n':1,'mean':mean,'ci95_low':mean,'ci95_high':mean}
 variance=sum((x-mean)**2 for x in values)/(len(values)-1);margin=1.96*math.sqrt(variance/len(values));return {'n':len(values),'mean':mean,'ci95_low':mean-margin,'ci95_high':mean+margin}
def summarize(records,group='planner'):
 metrics=('success','mission_duration_s','path_efficiency','collision_count','replan_count','mean_replan_time_ms','controller_cross_track_rmse_m');groups={}
 for record in records:groups.setdefault(str(record.get(group,'unknown')),[]).append(record)
 return {name:{metric:mean_ci([float(r[metric]) for r in rows if metric in r and r[metric] is not None]) for metric in metrics} for name,rows in sorted(groups.items())}
