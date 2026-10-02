"""ROS-independent ONNX terrain inference contract."""
from __future__ import annotations
import hashlib, json, time
from pathlib import Path

class InferenceContractError(RuntimeError): pass

def sha256(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): digest.update(block)
    return digest.hexdigest()

def load_contract(model_path,config_path,checksum_path=None):
    model_path,config_path=Path(model_path),Path(config_path)
    if not model_path.is_file(): raise InferenceContractError(f'model missing: {model_path}')
    if not config_path.is_file(): raise InferenceContractError(f'config missing: {config_path}')
    try: config=json.loads(config_path.read_text(encoding='utf-8'))
    except (OSError,json.JSONDecodeError) as exc: raise InferenceContractError(f'invalid config: {exc}') from exc
    if config.get('schema_version')!=1 or config.get('classes')!=list(range(9)):
        raise InferenceContractError('config must use schema 1 and contiguous class IDs 0..8')
    normalization=config.get('normalization',{})
    if not all(isinstance(normalization.get(key),list) and len(normalization[key])==3 for key in ('mean','std')):
        raise InferenceContractError('normalization mean/std must have three channels')
    if any(float(value)<=0 for value in normalization['std']): raise InferenceContractError('normalization std must be positive')
    size=config.get('data',{}).get('image_size')
    if not isinstance(size,list) or len(size)!=2 or any(not isinstance(x,int) or x<=0 for x in size):
        raise InferenceContractError('image_size must be [height,width]')
    if checksum_path:
        line=Path(checksum_path).read_text(encoding='utf-8').strip().split()
        if not line or line[0].lower()!=sha256(model_path): raise InferenceContractError('model checksum mismatch')
    return config

class OnnxTerrainInference:
    def __init__(self,model_path,config,device='cpu'):
        try:
            import onnxruntime as ort
        except ImportError as exc: raise InferenceContractError('onnxruntime is not installed') from exc
        available=ort.get_available_providers(); device=device.lower()
        if device not in ('cpu','cuda','auto'): raise InferenceContractError('device must be cpu, cuda, or auto')
        if device=='cuda' and 'CUDAExecutionProvider' not in available: raise InferenceContractError('CUDA provider unavailable')
        providers=(['CUDAExecutionProvider','CPUExecutionProvider'] if device in ('cuda','auto') and 'CUDAExecutionProvider' in available else ['CPUExecutionProvider'])
        self.session=ort.InferenceSession(str(model_path),providers=providers);self.config=config
        inputs,outputs=self.session.get_inputs(),self.session.get_outputs()
        if len(inputs)!=1 or len(outputs)!=1: raise InferenceContractError('model must have one image input and one logits output')
        self.input_name,self.output_name=inputs[0].name,outputs[0].name
    def infer(self,rgb):
        import numpy as np
        from PIL import Image
        if rgb.ndim!=3 or rgb.shape[2]!=3: raise InferenceContractError('expected HxWx3 RGB image')
        height,width=self.config['data']['image_size']; resized=np.asarray(Image.fromarray(rgb).resize((width,height),Image.Resampling.BILINEAR),dtype=np.float32)/255.0
        mean=np.asarray(self.config['normalization']['mean'],dtype=np.float32);std=np.asarray(self.config['normalization']['std'],dtype=np.float32)
        tensor=((resized-mean)/std).transpose(2,0,1)[None].astype(np.float32);start=time.perf_counter()
        logits=self.session.run([self.output_name],{self.input_name:tensor})[0];latency_ms=(time.perf_counter()-start)*1000
        if logits.ndim!=4 or logits.shape[0]!=1 or logits.shape[1]!=9: raise InferenceContractError(f'invalid logits shape: {logits.shape}')
        shifted=logits[0]-logits[0].max(axis=0,keepdims=True);prob=np.exp(shifted);prob/=prob.sum(axis=0,keepdims=True)
        labels=prob.argmax(axis=0).astype(np.uint8);confidence=prob.max(axis=0).astype(np.float32)
        thresholds=np.asarray([0.0]+[0.55,0.60,0.55,0.65,0.65,0.65,0.60,0.65],dtype=np.float32)
        labels[confidence < thresholds[labels]]=0
        return labels,confidence,latency_ms
