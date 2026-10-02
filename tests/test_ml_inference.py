#!/usr/bin/env python3
"""Phase 7 ROS ML inference contracts."""
import hashlib,json,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_perception'))
from lunabot_perception.inference_core import InferenceContractError,load_contract

def config():return {'schema_version':1,'classes':list(range(9)),'normalization':{'mean':[.1,.2,.3],'std':[.2,.2,.2]},'data':{'image_size':[64,64]}}
class MLInferenceTests(unittest.TestCase):
 def fixture(self,directory):
  root=Path(directory);model=root/'model.onnx';model.write_bytes(b'model');cfg=root/'model.config.json';cfg.write_text(json.dumps(config()));digest=hashlib.sha256(b'model').hexdigest();checksum=root/'model.sha256';checksum.write_text(f'{digest}  model.onnx\n');return model,cfg,checksum
 def test_contract_and_checksum(self):
  with tempfile.TemporaryDirectory() as directory:
   model,cfg,checksum=self.fixture(directory);self.assertEqual(load_contract(model,cfg,checksum)['classes'],list(range(9)))
   model.write_bytes(b'tampered')
   with self.assertRaisesRegex(InferenceContractError,'checksum'):load_contract(model,cfg,checksum)
 def test_missing_model_fails_safely(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);cfg=root/'config.json';cfg.write_text(json.dumps(config()))
   with self.assertRaisesRegex(InferenceContractError,'model missing'):load_contract(root/'absent.onnx',cfg)
   with self.assertRaisesRegex(InferenceContractError,'model_path parameter is empty'):load_contract('',cfg)
 def test_invalid_normalization_rejected(self):
  with tempfile.TemporaryDirectory() as directory:
   model,cfg,checksum=self.fixture(directory);bad=config();bad['normalization']['std']=[1,0,1];cfg.write_text(json.dumps(bad))
   with self.assertRaisesRegex(InferenceContractError,'positive'):load_contract(model,cfg)
 def test_ros_contract_is_installed(self):
  source=(ROOT/'src/lunabot_perception/lunabot_perception/terrain_inference_node.py').read_text();setup=(ROOT/'src/lunabot_perception/setup.py').read_text();launch=(ROOT/'src/lunabot_bringup/launch/perception.launch.py').read_text()
  for token in ('ApproximateTimeSynchronizer','desired_encoding=\'rgb8\'','mono8','32FC1','STATE_ERROR','if rclpy.ok()'):self.assertIn(token,source)
  self.assertIn('terrain_inference_node',setup)
  self.assertIn('numpy>=1.24,<2',(ROOT/'ml/requirements.txt').read_text())
  for mode in ('heuristic','ml','ground_truth'):self.assertIn(mode,launch)
if __name__=='__main__':unittest.main()
