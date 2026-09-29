import copy
from pathlib import Path
import tempfile
import unittest
from csim_experiments import fit_response, identify, synthetic_response
from csim_experiments.identification import read_response, write_response


class IdentificationTests(unittest.TestCase):
    def test_noiseless_identification_and_measured_csv(self):
        rows,config=synthetic_response(); report=fit_response(rows)
        self.assertEqual(report['delay_s'],config['truth']['delay_s'])
        for rotor,tau in zip(report['rotors'],config['truth']['time_constants_s']):
            self.assertAlmostEqual(rotor['time_constant_s'],tau,places=9)
            self.assertAlmostEqual(rotor['gain'],1,places=9)
            self.assertLess(rotor['validation_rmse_N'],1e-9)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'measured.csv'; write_response(path,rows)
            self.assertEqual(read_response(path),rows)
            self.assertEqual(identify(Path(directory)/'fit',input_csv=path),report)

    def test_noise_and_invalid_experiments(self):
        rows,config=synthetic_response(noise_std=.01,seed=42)
        self.assertEqual(rows,synthetic_response(noise_std=.01,seed=42)[0])
        report=fit_response(rows)
        self.assertEqual(report['delay_s'],.006)
        for rotor,tau in zip(report['rotors'],config['truth']['time_constants_s']):
            self.assertLess(abs(rotor['time_constant_s']-tau),.001)
            self.assertLess(rotor['validation_rmse_N'],.02)
        invalid=copy.deepcopy(rows); invalid[4]['time']=invalid[3]['time']
        with self.assertRaises(ValueError): fit_response(invalid)
        invalid=copy.deepcopy(rows); invalid[-1]['command_0']=10
        with self.assertRaises(ValueError): fit_response(invalid)
        invalid=copy.deepcopy(rows)
        for row in invalid: row['command_3']=0
        with self.assertRaises(ValueError): fit_response(invalid)
        invalid=copy.deepcopy(rows)
        for row in invalid:
            for i in range(4): row[f'thrust_{i}']=0
        with self.assertRaises(ValueError): fit_response(invalid)
        with self.assertRaises(ValueError): fit_response(rows[:20])

if __name__=='__main__': unittest.main()
