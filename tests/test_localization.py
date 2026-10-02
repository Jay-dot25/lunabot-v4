#!/usr/bin/env python3
"""Phase 8 GPS-denied localization tests."""
import re,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/lunabot_localization'))
from lunabot_localization.metrics import evaluate_trajectory
class LocalizationTests(unittest.TestCase):
 def test_perfect_trajectory(self):
  trajectory=[(0,0,0,0),(1,1,0,0),(2,2,0,0)];result=evaluate_trajectory(trajectory,trajectory)
  self.assertEqual(result['ate_rmse_m'],0);self.assertEqual(result['drift_per_m'],0)
 def test_known_drift_and_rpe(self):
  truth=[(0,0,0,0),(1,1,0,0),(2,2,0,0)];estimated=[(0,0,0,0),(1,1.1,0,0),(2,2.4,0,0)];result=evaluate_trajectory(estimated,truth)
  self.assertAlmostEqual(result['final_pose_error_m'],.4);self.assertAlmostEqual(result['drift_per_m'],.2);self.assertGreater(result['rpe_rmse_m'],0)
 def test_rejects_unaligned_data(self):
  with self.assertRaisesRegex(ValueError,'timestamps'):evaluate_trajectory([(0,0,0,0),(2,1,0,0)],[(0,0,0,0),(1,1,0,0)])
 def test_ekf_is_gps_denied_and_tf_is_unambiguous(self):
  ekf=(ROOT/'src/lunabot_localization/config/ekf.yaml').read_text();slam=(ROOT/'src/lunabot_localization/config/slam_toolbox_filtered.yaml').read_text()
  self.assertIn('odom0: /lunabot/odometry',ekf);self.assertIn('imu0: /lunabot/imu',ekf);self.assertIn('publish_tf: true',ekf);self.assertIn('world_frame: odom',ekf);self.assertNotIn('ground_truth',ekf.lower());self.assertIn('odom_frame: odom',slam);self.assertIn('base_frame: chassis',slam)
  for name in ('process_noise_covariance','initial_estimate_covariance'):
   values=re.search(rf'{name}: \[([^]]+)\]',ekf).group(1).split(',')
   self.assertEqual(len(values),225);self.assertTrue(all('.' in value for value in values))
 def test_noise_and_evaluation_boundary(self):
  sdf=(ROOT/'src/lunabot_gazebo/models/lunabot_v4/model.sdf').read_text();source=(ROOT/'src/lunabot_localization/lunabot_localization/localization_evaluator.py').read_text()
  self.assertGreaterEqual(sdf.count('noise type="gaussian"'),7);self.assertIn('/lunabot/ground_truth/pose',source);self.assertNotIn('create_publisher',source)
if __name__=='__main__':unittest.main()
