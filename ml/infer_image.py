#!/usr/bin/env python3
"""Run reproducible ONNX inference on one RGB image."""
import argparse,json
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--image',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 import numpy as np
 import onnxruntime as ort
 from PIL import Image
 c=json.loads(a.config.read_text());h,w=c['data']['image_size'];image=Image.open(a.image).convert('RGB').resize((w,h),Image.Resampling.BILINEAR);array=np.asarray(image,dtype=np.float32).transpose(2,0,1)/255;array=(array-np.asarray(c['normalization']['mean'])[:,None,None])/np.asarray(c['normalization']['std'])[:,None,None]
 logits=ort.InferenceSession(str(a.model),providers=['CPUExecutionProvider']).run(None,{'image':array[None].astype(np.float32)})[0];mask=logits.argmax(1)[0].astype(np.uint8);a.output.parent.mkdir(parents=True,exist_ok=True);Image.fromarray(mask).save(a.output);print(f'INFERENCE_PASS output={a.output}')
if __name__=='__main__':main()
