#!/usr/bin/env python3
"""Evaluate only the held-out world split and emit auditable metrics."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from lunabot_ml.config import config_sha256,load_config,seed_everything
from lunabot_ml.dataset import create_dataset
from lunabot_ml.metrics import confusion_matrix,merge_confusion,report
from lunabot_ml.models import build_model

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--device',default='cpu');a=p.parse_args()
 import torch
 from torch.utils.data import DataLoader
 c=load_config(a.config);seed_everything(c['training']['seed']); checkpoint=torch.load(a.checkpoint,map_location='cpu',weights_only=True)
 if checkpoint.get('config_sha256')!=config_sha256(c): raise SystemExit('checkpoint/config mismatch')
 model=build_model(c);model.load_state_dict(checkpoint['state_dict']);model.to(a.device).eval(); data=create_dataset(a.dataset,'test',c,False);loader=DataLoader(data,batch_size=1,shuffle=False)
 matrix=[[0]*9 for _ in range(9)];elapsed=0.0
 with torch.no_grad():
  for images,labels,_ in loader:
   start=time.perf_counter();pred=model(images.to(a.device)).argmax(1).cpu();elapsed+=time.perf_counter()-start
   matrix=merge_confusion(matrix,confusion_matrix(pred.flatten().tolist(),labels.flatten().tolist()))
 names=['unknown','flat_regolith','rough_regolith','bedrock','small_rock','large_rock','crater','shadow','habitat']; result=report(matrix,names);result.update({'schema_version':1,'split':'test','samples':len(data),'latency_ms':1000*elapsed/max(1,len(data)),'fps':len(data)/elapsed if elapsed else 0,'config_sha256':config_sha256(c)})
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n');print(f"EVALUATION_COMPLETE samples={len(data)} mean_iou={result['mean_iou']:.4f}")
if __name__=='__main__':main()
