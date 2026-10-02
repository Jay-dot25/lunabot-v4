#!/usr/bin/env python3
"""Phase 6 dependency-light ML framework tests."""
import json,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ml'))
from lunabot_ml.config import config_sha256,load_config
from lunabot_ml.metrics import confusion_matrix,merge_confusion,report

class MLFrameworkTests(unittest.TestCase):
 def test_all_architecture_configs(self):
  for name in ('unet','deeplabv3plus','segformer_b0'):
   config=load_config(ROOT/f'ml/configs/{name}.yaml');self.assertEqual(config['model']['architecture'],name);self.assertEqual(len(config_sha256(config)),64)
 def test_config_rejects_unknown_architecture(self):
  source=json.loads((ROOT/'ml/configs/unet.yaml').read_text());source['model']['architecture']='imaginary'
  with tempfile.TemporaryDirectory() as directory:
   path=Path(directory)/'bad.json';path.write_text(json.dumps(source))
   with self.assertRaisesRegex(ValueError,'architecture'):load_config(path)
 def test_confusion_and_safety_metrics(self):
  matrix=confusion_matrix([1,1,2,6],[1,5,1,6]);result=report(matrix)
  self.assertEqual(matrix[5][1],1);self.assertAlmostEqual(result['hazard_false_negative_rate'],0.5);self.assertGreater(result['mean_iou'],0)
 def test_merge_confusion(self):
  matrix=confusion_matrix([1,2],[1,2]);self.assertEqual(merge_confusion(matrix,matrix)[1][1],2)
 def test_entry_points_and_artifact_policy(self):
  for name in ('train.py','evaluate.py','export_onnx.py','infer_image.py'):
   self.assertTrue((ROOT/'ml'/name).is_file())
  requirements=(ROOT/'ml/requirements.txt').read_text()
  self.assertIn('onnxscript',requirements)
  exporter=(ROOT/'ml/export_onnx.py').read_text()
  self.assertIn('external_data=False',exporter)
  readme=(ROOT/'models/README.md').read_text();self.assertIn('not committed',readme);self.assertIn('checksum',readme)
if __name__=='__main__':unittest.main()
