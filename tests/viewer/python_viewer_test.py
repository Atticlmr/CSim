"""Run explicitly in a graphical context (or Xvfb), separate from headless tests."""
import tempfile
import math
import threading
import unittest
from pathlib import Path

import csim
from csim_control import ControlConfig, ControlLoop
from csim_viewer import Viewer


class ViewerTests(unittest.TestCase):
    def test_models_sync_without_physics_changes(self):
        for payload in (False, True):
            model = csim.SuspendedPayloadModel(timestep=0.002) if payload else csim.DroneModel(timestep=0.002)
            thrust = ((model.drone_mass+model.payload_mass) if payload else model.mass)*model.gravity
            kwargs = dict(position_W=[0, 0, 5])
            if payload:
                kwargs['thrust'] = thrust
            data = csim.make_data(model, **kwargs)
            reference = csim.make_data(model, **kwargs)
            csim.set_control(model, data, thrust=thrust)
            csim.set_control(model, reference, thrust=thrust)
            with Viewer(model, width=800, height=600, hidden=True) as viewer:
                before = csim.get_state(model, data)
                viewer.sync(data)
                self.assertEqual(before, csim.get_state(model, data))
                for i in range(100):
                    csim.step(model, data)
                    csim.step(model, reference)
                    if i % 7 == 0:
                        viewer.sync(data)
                self.assertEqual(csim.get_state(model, data), csim.get_state(model, reference))
                viewer.sync(data)
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory)/'image.ppm'
                    viewer.screenshot(path)
                    image = path.read_bytes()
                    self.assertTrue(image.startswith(b'P6\n800 600\n255\n'))
                    self.assertGreater(len(set(image[-800*600*3:])), 50)
            self.assertFalse(viewer.is_running())
            viewer.close()
            with self.assertRaises(RuntimeError):
                viewer.sync(data)

    def test_imported_urdf_mjcf_geometry_matches(self):
        root=Path(__file__).resolve().parents[2]
        for directory,stem in [('tests/fixtures/model_import','fixed_frame'),('examples/models','drone')]:
            images=[]
            for suffix in ('urdf','xml'):
                model=csim.load_model(str(root/directory/(stem+'.'+suffix)),free_base=True)
                data=csim.make_data(model,position_W=[0,0,5])
                with Viewer(model,width=800,height=600,hidden=True) as viewer:
                    viewer.sync(data)
                    with tempfile.TemporaryDirectory() as tmp:
                        path=Path(tmp)/'frame.ppm'; viewer.screenshot(path)
                        images.append(path.read_bytes())
            self.assertEqual(len(images[0]),len(images[1]))
            self.assertLess(sum(abs(a-b) for a,b in zip(*images))/len(images[0]),.05)

    def test_ctbr_render_cadence_does_not_change_result(self):
        control=ControlConfig(
            mode='ctbr',command_delay=.004,controller_period=.004,time_constants=[.02]*4,
            allocation_mode='rotor',max_rotor_thrust=[8]*4)
        for method in ("rk4","dopri5"):
            model=csim.SuspendedPayloadModel(timestep=.002,integrator=method,
                drone_drag=csim.DragConfig(k1=.1),payload_drag=csim.DragConfig(k1=.02,k0=.001,sign_mode='tanh',epsilon_v=.02),
                wind=csim.WindField(velocity_W=[1,0,0],gust_amplitude_W=[0,.2,0],gust_frequency=.7))
            results=[]
            for cadence in (0,3,17):
                data=csim.make_data(model,thrust=1.2*model.gravity,position_W=[0,0,5])
                loop=ControlLoop(model,data,control)
                with Viewer(model,width=640,height=480,hidden=True) as viewer:
                    for i in range(200):
                        if i%5==0: loop.set_ctbr(1.2*model.gravity,[0,.03*math.sin(i*.01),0])
                        loop.step()
                        if cadence and i%cadence==0: viewer.sync(data)
                    results.append(csim.get_state(model,data))
            self.assertEqual(results[0],results[1]); self.assertEqual(results[0],results[2])

    def test_rigid_payload_geometry_and_render_independence(self):
        root = Path(__file__).resolve().parents[2] / 'examples/models/rigid_payload'
        images = []
        for extension in ('urdf', 'xml'):
            model = csim.load_suspended_model(str(root/f'drone.{extension}'), str(root/'cable.json'), str(root/f'payload.{extension}'))
            data = csim.make_data(model, thrust=15, position_W=[0, 0, 5])
            reference = csim.make_data(model, thrust=15, position_W=[0, 0, 5])
            with Viewer(model, width=800, height=600, hidden=True) as viewer:
                initial = csim.get_state(model, data)
                viewer.sync(data)
                self.assertEqual(initial, csim.get_state(model, data))
                for index in range(100):
                    csim.step(model, data)
                    csim.step(model, reference)
                    if index % 7 == 0:
                        viewer.sync(data)
                self.assertEqual(csim.get_state(model, data), csim.get_state(model, reference))
                viewer.sync(data)
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory)/'frame.ppm'
                    viewer.screenshot(path)
                    images.append(path.read_bytes())
                    before = images[-1]
                    csim.reset(model, data, thrust=15, position_W=[0, 0, 5], payload_q_WP=[math.sqrt(.5), 0, 0, math.sqrt(.5)])
                    viewer.sync(data)
                    viewer.screenshot(path)
                    self.assertNotEqual(before, path.read_bytes())
                    csim.reset(model, data, mode='slack', position_W=[0, 0, 5])
                    viewer.sync(data)
        self.assertEqual(images[0], images[1])

    def test_identity_lifetime_and_thread_rules(self):
        model = csim.DroneModel()
        data = csim.make_data(model)
        wrong = csim.make_data(csim.DroneModel())
        with self.assertRaisesRegex(RuntimeError, 'exception cleanup'):
            with Viewer(model, hidden=True) as viewer:
                with self.assertRaises(ValueError):
                    viewer.sync(wrong)
                with self.assertRaises(RuntimeError):
                    Viewer(model, hidden=True)
                viewer.sync(data)
                errors = []
                def worker():
                    try:
                        viewer.close()
                    except RuntimeError as error:
                        errors.append(str(error))
                thread = threading.Thread(target=worker)
                thread.start(); thread.join()
                self.assertEqual(len(errors), 1)
                raise RuntimeError('exception cleanup')
        with Viewer(model, hidden=True) as viewer:
            del model
            viewer.sync(data)


if __name__ == '__main__':
    unittest.main()
