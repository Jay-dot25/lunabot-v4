#!/usr/bin/env python3
"""Generate or execute a reproducible factorial experiment matrix."""
import argparse,json,shlex,subprocess,time
from pathlib import Path
from experiment_tools import load_manifest
def main():
 p=argparse.ArgumentParser();p.add_argument('manifest');p.add_argument('--output',default='evidence/experiments');p.add_argument('--execute',action='store_true');p.add_argument('--resume',action='store_true');args=p.parse_args();config,trials=load_manifest(args.manifest);root=Path(args.output);root.mkdir(parents=True,exist_ok=True);(root/'resolved_manifest.json').write_text(json.dumps({'source':str(args.manifest),'trials':trials},indent=2,sort_keys=True)+'\n');failures=0
 for trial in trials:
  directory=root/trial['trial_id'];directory.mkdir(exist_ok=True);metadata=directory/'metadata.json';result=directory/'trial.json'
  if args.resume and result.exists():continue
  metadata.write_text(json.dumps(trial,indent=2,sort_keys=True)+'\n')
  command=str(config.get('command','')).format(output=directory,**trial);print(f"{trial['trial_id']}: {command or '[no command]'}")
  if args.execute:
   if not command:raise SystemExit('manifest command required with --execute')
   started=time.time();completed=subprocess.run(shlex.split(command),timeout=float(config.get('timeout_seconds',900)),check=False);metadata.write_text(json.dumps({**trial,'command':command,'started_unix':started,'returncode':completed.returncode},indent=2,sort_keys=True)+'\n');failures+=completed.returncode!=0
 return 1 if failures else 0
if __name__=='__main__':raise SystemExit(main())
