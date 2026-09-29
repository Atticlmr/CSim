"""Fixed-link kinematics across file formats, frames and simulation adapters."""
import math
from pathlib import Path
import tempfile
import unittest

import csim


FIXTURES = Path(__file__).resolve().parents[1] / 'fixtures/model_import'
Q90 = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]


class LinkStateTests(unittest.TestCase):
    def near(self, actual, expected, tolerance=1e-10):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def model(self, suffix='urdf', **options):
        return csim.load_model(str(FIXTURES / f'fixed_frame.{suffix}'), free_base=True, **options)

    def test_source_frames_and_link_centers_survive_aggregation(self):
        for suffix in ('urdf', 'xml'):
            for axes in ([1, 0, 0, 0], Q90):
                with self.subTest(suffix=suffix, axes=axes):
                    model = self.model(suffix, root_to_body=axes)
                    data = csim.make_data(model)
                    self.assertEqual(model.link_names, ['base', 'arm'])
                    states = csim.get_link_states(model, data)
                    self.assertEqual(list(states), model.link_names)
                    self.assertIsNone(states['base']['parent'])
                    self.assertEqual(states['arm']['parent'], 'base')
                    self.near(states['base']['position_W'], [0, 0, 0])
                    self.near(states['base']['com_position_W'], [.1, 0, 0])
                    self.near(states['base']['q_WL'], [1, 0, 0, 0])
                    self.near(states['arm']['position_W'], [.4, 0, .1])
                    self.near(states['arm']['com_position_W'], [.3, 0, .1])
                    self.near(states['arm']['q_WL'], Q90)
                    self.near(states['arm']['acceleration_W'], [0, 0, -model.gravity])
                    self.assertEqual(states['arm'], csim.get_link_state(model, data, 'arm'))

    def test_nested_link_without_visuals_and_source_root_placement(self):
        urdf = (FIXTURES / 'fixed_frame.urdf').read_text().replace('</robot>', '''
<link name="sensor"><inertial><mass value="0.1"/>
<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial></link>
<joint name="sensor_mount" type="fixed"><parent link="arm"/><child link="sensor"/>
<origin xyz="0.2 0 0" rpy="1.5707963267948966 0 0"/></joint></robot>''')
        xml = (FIXTURES / 'fixed_frame.xml').read_text().replace(
            '      </body>', '''
        <body name="sensor" pos="0.2 0 0" quat="0.7071067811865476 0.7071067811865476 0 0">
          <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>
        </body>
      </body>''')
        with tempfile.TemporaryDirectory() as directory:
            for suffix, source in (('urdf', urdf), ('xml', xml)):
                path = Path(directory) / f'nested.{suffix}'
                path.write_text(source)
                model = csim.load_model(str(path), free_base=True, root_to_body=Q90)
                data = csim.make_data(model)
                sensor = csim.get_link_state(model, data, 'sensor')
                self.assertEqual(sensor['parent'], 'arm')
                self.near(sensor['position_W'], [.4, .2, .1])
                self.near(sensor['q_WL'], [.5, .5, .5, .5])
                self.near(sensor['position_W'], sensor['com_position_W'])

            # Source root pose and root_to_body must each be applied exactly once.
            path.write_text(xml.replace('<body name="base">',
                '<body name="base" pos="1 2 3" quat="0.7071067811865476 0 0 0.7071067811865476">'))
            model = csim.load_model(str(path), free_base=True, root_to_body=Q90)
            data = csim.make_data(model)
            self.near(csim.get_link_state(model, data, 'sensor')['position_W'], [.8, 2.4, 3.1])
            self.near(csim.get_link_state(model, data, 'base')['q_WL'], Q90)
            csim.reset(model, data, position_W=[5, 6, 7])
            csim.reset(model, data)
            self.near(csim.get_link_state(model, data, 'sensor')['position_W'], [.8, 2.4, 3.1])

    def test_rotation_velocity_and_tangential_and_centripetal_acceleration(self):
        # Analytic assembly tensor is [[.036,0,-.008],[0,.041,0],[-.008,0,.059]].
        # For omega_B=[0,0,2], this torque gives alpha_B=[.3,-.2,.4].
        torque = [.0076, -.0402, .0212]
        for suffix in ('urdf', 'xml'):
            for payload in (False, True):
                with self.subTest(suffix=suffix, payload=payload):
                    options = {'payload_mass': .4} if payload else {}
                    model = self.model(suffix, timestep=0.0001, **options)
                    mass = model.drone_mass + model.payload_mass if payload else model.mass
                    initial = dict(position_W=[1, 2, 3], velocity_W=[.4, .5, .6],
                                   q_WB=Q90, angular_velocity_B=[0, 0, 2])
                    if payload:
                        initial.update(thrust=mass*model.gravity, torque_B=torque)
                    data = csim.make_data(model, **initial)
                    csim.set_control(model, data, mass*model.gravity, torque)
                    arm = csim.get_link_state(model, data, 'arm')
                    self.near(arm['position_W'], [1, 2.26, 3.08])
                    self.near(arm['q_WL'], [0, 0, 0, 1])
                    self.near(arm['velocity_W'], [-.12, .5, .6])
                    self.near(arm['com_velocity_W'], [.08, .5, .6])
                    self.near(arm['angular_velocity_W'], [0, 0, 2])
                    self.near(arm['angular_acceleration_W'], [.2, .3, .4])
                    self.near(arm['acceleration_W'], [-.08, -1.056, .052])
                    self.near(arm['com_acceleration_W'], [-.04, -.656, .032])
                    # Central differences after real physical steps validate derivative consistency.
                    samples = [arm]
                    for _ in range(2):
                        csim.step(model, data)
                        samples.append(csim.get_link_state(model, data, 'arm'))
                    for prefix in ('', 'com_'):
                        for field, derivative in (('position_W', 'velocity_W'),
                                                  ('velocity_W', 'acceleration_W')):
                            finite_difference = [(b-a)/(2*model.timestep) for a, b in zip(
                                samples[0][prefix+field], samples[2][prefix+field])]
                            self.near(finite_difference, samples[1][prefix+derivative], 1e-6)

    def test_angular_velocity_is_expressed_in_world_axes(self):
        model = self.model()
        data = csim.make_data(model, q_WB=[math.sqrt(.5), math.sqrt(.5), 0, 0],
                              angular_velocity_B=[0, 0, 2])
        arm = csim.get_link_state(model, data, 'arm')
        self.near(arm['angular_velocity_W'], [0, -2, 0])
        self.near(arm['velocity_W'], [0, 0, .52])

    def test_read_only_snapshots_ownership_and_missing_names(self):
        model = self.model()
        data = csim.make_data(model)
        initial = csim.get_state(model, data)
        names = model.link_names
        names.clear()
        self.assertEqual(model.link_names, ['base', 'arm'])
        links = csim.get_link_states(model, data)
        links['arm']['position_W'][0] = 100
        links['base']['com_velocity_W'][0] = 100
        self.near(csim.get_link_state(model, data, 'arm')['position_W'], [.4, 0, .1])
        self.assertEqual(initial, csim.get_state(model, data))
        with self.assertRaises(KeyError):
            csim.get_link_state(model, data, 'missing')
        other = self.model()
        with self.assertRaisesRegex(ValueError, 'different model'):
            csim.get_link_states(other, data)
        with self.assertRaisesRegex(ValueError, 'different model'):
            csim.get_link_state(other, data, 'arm')
        csim.set_control(model, data, model.mass*model.gravity)
        self.near(csim.get_link_state(model, data, 'base')['acceleration_W'], [0, 0, 0])
        csim.step(model, data)
        self.assertEqual(csim.get_link_state(model, data, 'arm')['time'], model.timestep)
        csim.reset(model, data)
        self.assertEqual(csim.get_link_state(model, data, 'arm')['time'], 0)

    def test_models_without_imported_links_and_separate_point_payload(self):
        for model in (csim.DroneModel(), csim.SuspendedPayloadModel()):
            options = {} if isinstance(model, csim.DroneModel) else {
                'thrust': (model.drone_mass+model.payload_mass)*model.gravity}
            data = csim.make_data(model, **options)
            self.assertEqual(model.link_names, [])
            self.assertEqual(csim.get_link_states(model, data), {})
            with self.assertRaises(KeyError):
                csim.get_link_state(model, data, 'base')
        model = self.model(payload_mass=.4)
        data = csim.make_data(model, thrust=(model.drone_mass+model.payload_mass)*model.gravity)
        self.assertEqual(model.link_names, ['base', 'arm'])
        with self.assertRaises(KeyError):
            csim.get_link_state(model, data, 'payload')
        self.assertIn('payload_position_W', csim.get_state(model, data))


if __name__ == '__main__':
    unittest.main()
