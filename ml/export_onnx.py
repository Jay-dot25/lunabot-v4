#!/usr/bin/env python3
"""Export a checkpoint to ONNX with config and SHA-256 sidecars."""
import argparse,hashlib,json,shutil,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from lunabot_ml.config import config_sha256,load_config
from lunabot_ml.models import build_model

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,default=Path('models/terrain_segmentation.onnx'));a=p.parse_args()
 import torch
 c=load_config(a.config); checkpoint=torch.load(a.checkpoint,map_location='cpu',weights_only=True)
 if checkpoint.get('config_sha256')!=config_sha256(c): raise SystemExit('checkpoint/config mismatch')
 model=build_model(c);model.load_state_dict(checkpoint['state_dict']);model.eval();h,w=c['data']['image_size'];a.output.parent.mkdir(parents=True,exist_ok=True)
 torch.onnx.export(model,torch.zeros(1,3,h,w),a.output,input_names=['image'],output_names=['logits'],opset_version=18,external_data=False,dynamic_axes={'image':{0:'batch'},'logits':{0:'batch'}})
 config_path=a.output.with_suffix('.config.json');config_path.write_text(json.dumps(c,indent=2,sort_keys=True)+'\n');digest=hashlib.sha256(a.output.read_bytes()).hexdigest();a.output.with_suffix('.sha256').write_text(f'{digest}  {a.output.name}\n');print(f'ONNX_EXPORT_PASS sha256={digest}')
if __name__=='__main__':main()
