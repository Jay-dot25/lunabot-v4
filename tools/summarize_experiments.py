#!/usr/bin/env python3
"""Summarize trial JSON with per-planner 95% confidence intervals."""
import argparse,csv,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent));from experiment_tools import summarize
def main():
 p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--output');args=p.parse_args();root=Path(args.directory);records=[]
 for path in sorted(root.glob('trial-*/trial.json')):
  record=json.loads(path.read_text());meta=path.parent/'metadata.json'
  if meta.exists():record={**json.loads(meta.read_text()),**record}
  records.append(record)
 summary=summarize(records);output=Path(args.output or root/'summary.json');output.write_text(json.dumps({'trial_count':len(records),'groups':summary},indent=2,sort_keys=True)+'\n');csv_path=output.with_suffix('.csv')
 with csv_path.open('w',newline='') as f:
  writer=csv.writer(f);writer.writerow(['planner','metric','n','mean','ci95_low','ci95_high'])
  for planner,metrics in summary.items():
   for metric,stats in metrics.items():writer.writerow([planner,metric,stats['n'],stats['mean'],stats['ci95_low'],stats['ci95_high']])
 print(f'EXPERIMENT_SUMMARY trials={len(records)} groups={len(summary)} output={output}');return 0 if records else 1
if __name__=='__main__':raise SystemExit(main())
