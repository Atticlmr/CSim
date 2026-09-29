import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
import csim
from csim_control import ControlConfig
from csim_experiments import default_config, default_sweep_config, model_from_config, run_flight, replay, run_sweep
from csim_experiments.flight import initial_data
from csim_experiments.recording import read_json, write_json


class ExperimentTests(unittest.TestCase):
    def test_physical_configuration_roundtrip_and_independent_copy(self):
        for payload in (False,True):
            config=default_config('tracking',rotors=True,payload=payload)
            config['model']['wind'].update(velocity_W=[1,2,3],gradient_W=[[0,0,.1],[0,0,0],[0,0,0]])
            config['model']['drone_drag']['k2']=.1
            if payload: config['model']['payload_drag'].update(k0=.001,sign_mode='tanh',epsilon_v=.02)
            first=model_from_config(config['model']); second=model_from_config(csim.get_config(first))
            self.assertEqual(csim.get_config(first),csim.get_config(second))
            changed=csim.get_config(first); changed['inertia_B'][0][0]=1000
            self.assertNotEqual(changed,csim.get_config(first))
            self.assertEqual(csim.get_state(first,initial_data(first,config['model'],config['initial'])),
                             csim.get_state(second,initial_data(second,config['model'],config['initial'])))
        root=Path(__file__).resolve().parents[2]
        imported=csim.load_model(str(root/'tests/fixtures/model_import/fixed_frame.xml'),free_base=True)
        config=csim.get_config(imported); rebuilt=model_from_config(config)
        self.assertEqual(csim.get_state(imported,csim.make_data(imported)),
                         csim.get_state(rebuilt,initial_data(rebuilt,config,{})))

    def test_imported_initial_pose_survives_flight_configuration(self):
        config=default_config('hover'); config['experiment']['duration']=.02
        config['initial']={}; config['model']['initial_pose_WB']['position_W']=[1,2,3]
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'run'; run_flight(config,output)
            self.assertEqual(read_json(output/'config.json')['initial_state']['position_W'],[1,2,3])
            self.assertEqual(replay(output)['accepted_steps'],10)

    def test_full_records_replay_metrics_and_no_overwrite(self):
        config=default_sweep_config(); config['experiment']['duration']=.2
        config['model']['wind']['velocity_W']=[2,0,0]
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'run'; result=run_flight(config,output)
            recorded=read_json(output/'config.json')
            self.assertEqual(recorded['model'],config['model']); self.assertIn('controller',recorded['experiment'])
            self.assertEqual(len(recorded['provenance']['native_sha256']),64)
            rows=[json.loads(line) for line in (output/'states.jsonl').read_text().splitlines()]
            commands=[json.loads(line) for line in (output/'commands.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows),101); self.assertEqual([r['step'] for r in commands],list(range(0,100,5)))
            self.assertEqual(rows[-1]['state'],result['state']); self.assertIn('aerodynamics',rows[-1]['state'])
            verified=replay(output); self.assertEqual(verified['verified_states'],101)
            self.assertTrue(verified['same_native_binary'])
            errors=[sum((a-b)**2 for a,b in zip(row['state']['position_W'],row['reference']['position_W'])) for row in rows]
            integral=sum((errors[i-1]+errors[i])*(rows[i]['state']['time']-rows[i-1]['state']['time'])/2 for i in range(1,len(rows)))
            self.assertAlmostEqual(result['metrics']['position_rmse_m'],math.sqrt(integral/result['state']['time']),places=14)
            other=run_flight(config,Path(directory)/'other'); self.assertEqual(result['state'],other['state'])
            with self.assertRaises(FileExistsError): run_flight(config,output)
            rows[1]['state']['position_W'][0]+=.001
            (output/'states.jsonl').write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
            with self.assertRaisesRegex(ValueError,'position_W'): replay(output)

    def test_strict_json_and_failure_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'; config=ControlConfig().to_dict()
            write_json(path,config)
            self.assertIn('"+inf"',path.read_text()); self.assertNotIn('Infinity',path.read_text())
            self.assertEqual(read_json(path),config)
            path.write_text('{"bad":NaN}')
            with self.assertRaises(ValueError): read_json(path)
            config=default_sweep_config(); config['experiment']['duration']=.2
            config['model']['payload_drag'].update(k1=.1,k2=0,k0=0,sign_mode='exact',epsilon_v=0)
            config['model']['wind'].update(gust_amplitude_W=[0,0,500],gust_frequency=1.)
            output=Path(directory)/'failure'
            with self.assertRaises(ValueError): run_flight(config,output)
            result=read_json(output/'result.json'); self.assertEqual(result['status'],'failed')
            self.assertEqual(result['error']['stage'],'step'); self.assertGreater(result['metrics']['accepted_steps'],0)
            self.assertTrue(replay(output)['failure_reproduced'])

    def test_sweep_cartesian_grid_wind_effect_and_rejected_delay(self):
        config=default_sweep_config(); config['experiment']['duration']=.1
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'sweep'
            results=run_sweep(config,output,timesteps=[.002],payload_masses=[.2,.4],delays=[0,.003],wind_speeds=[0,2])
            self.assertEqual(len(results),8)
            good=[r for r in results if r['status']=='completed']; self.assertEqual(len(good),4)
            self.assertNotEqual(good[0]['position_rmse_m'],good[1]['position_rmse_m'])
            for row in good: self.assertEqual(replay(output/row['run'])['accepted_steps'],50)
            failed=[r for r in results if r['status']=='failed']
            for row in failed: self.assertTrue((output/row['run']/'rejected_config.json').exists())
            self.assertEqual(len((output/'summary.csv').read_text().splitlines()),9)

    def test_outer_controller_configuration_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            config=default_config(); config['experiment']['duration']=.1
            for key,value in [('position_gain',float('nan')),('acceleration_limits',[3,3,20]),('unknown',1)]:
                invalid=copy.deepcopy(config); invalid['experiment']['controller'][key]=value
                with self.assertRaises(ValueError): run_flight(invalid,Path(directory)/'bad')
                self.assertFalse((Path(directory)/'bad').exists())

if __name__=='__main__': unittest.main()
