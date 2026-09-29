import math
from pathlib import Path
import struct
import tempfile
import unittest
import csim

FIXTURES = Path(__file__).resolve().parents[1] / 'fixtures/model_import'

class ModelImportTests(unittest.TestCase):
    def test_fixed_assembly_inertia_and_pose(self):
        for suffix in ('urdf', 'xml'):
            path = str(FIXTURES / ('fixed_frame.' + suffix))
            with self.assertRaisesRegex(ValueError, 'free_base'):
                csim.load_model(path)
            m = csim.load_model(path, free_base=True)
            self.assertEqual(m.mass, 2.5)
            for actual, expected in zip(m.inertia_B, ((.036,0,-.008),(0,.041,0),(-.008,0,.059))):
                for a,b in zip(actual,expected): self.assertAlmostEqual(a,b,places=12)
            d = csim.make_data(m)
            for a,b in zip(csim.get_state(m,d)['position_W'],(.14,0,.02)):
                self.assertAlmostEqual(a,b,places=12)
            csim.set_control(m,d,m.mass*m.gravity)
            initial = csim.get_state(m,d)['position_W']
            for _ in range(100): csim.step(m,d)
            self.assertEqual(initial,csim.get_state(m,d)['position_W'])
            csim.reset(m,d,position_W=[0,0,5])
            csim.reset(m,d)
            self.assertEqual(initial,csim.get_state(m,d)['position_W'])

    def test_axes_and_payload_adapter(self):
        path = str(FIXTURES/'fixed_frame.urdf')
        q=[math.sqrt(.5),0,0,math.sqrt(.5)]
        m=csim.load_model(path,free_base=True,root_to_body=q,payload_mass=.4)
        self.assertAlmostEqual(m.inertia_B[0][0],.041)
        self.assertAlmostEqual(m.inertia_B[1][2],-.008)
        d=csim.make_data(m,thrust=(m.drone_mass+m.payload_mass)*m.gravity)
        self.assertAlmostEqual(csim.get_state(m,d)['q_WB'][3],-q[3])
        self.assertAlmostEqual(csim.get_state(m,d)['tension'],.4*m.gravity)

    def test_strict_failures(self):
        urdf=(FIXTURES/'fixed_frame.urdf').read_text()
        xml=(FIXTURES/'fixed_frame.xml').read_text()
        changes=[(urdf,'urdf','type="fixed"','type="revolute"'),
                 (urdf,'urdf','mass value="2"','mass value="nan"'),
                 (urdf,'urdf','ixx="0.02"','ixx="20"'),
                 (urdf,'urdf','<robot ','<!DOCTYPE robot><robot '),
                 (xml,'xml','inertiafromgeom="false"','inertiafromgeom="true"'),
                 (xml,'xml','contype="0"','contype="1"'),
                 (urdf,'urdf','<child link="arm"/>','<child link="absent"/>'),
                 (urdf,'urdf','<parent link="base"/>','<parent link="arm"/>'),
                 (urdf,'urdf','name="arm"','name="base"'),
                 (xml,'xml','<worldbody>','<default/><worldbody>'),
                 (xml,'xml','<worldbody>','<include file="missing.xml"/><worldbody>')]
        with tempfile.TemporaryDirectory() as directory:
            for original,suffix,old,new in changes:
                self.assertIn(old,original)
                p=Path(directory)/('bad.'+suffix); p.write_text(original.replace(old,new))
                with self.assertRaises(ValueError): csim.load_model(str(p),free_base=True)

    def test_mesh_resources_and_missing_inertia(self):
        original=(FIXTURES/'fixed_frame.urdf').read_text()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'triangle.obj').write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n')
            stl=b'CSim'.ljust(80,b'\0')+struct.pack('<I12fH',1,0,0,1,0,0,0,1,0,0,0,1,0,0)
            (root/'triangle.stl').write_bytes(stl)
            (root/'ascii.stl').write_text('solid a\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid a\n')
            for mesh in ('triangle.obj','triangle.stl','ascii.stl'):
                text=original.replace('<box size="0.4 0.2 0.1"/>',f'<mesh filename="package://test/{mesh}" scale="0.1 0.2 0.3"/>')
                self.assertNotEqual(text,original)
                p=root/'mesh.urdf'; p.write_text(text)
                with self.assertRaisesRegex(ValueError,'package'): csim.load_model(str(p),free_base=True)
                m=csim.load_model(str(p),free_base=True,package_roots={'test':directory})
                self.assertEqual(m.mass,2.5)
            p.write_text('<robot name="empty"><link name="base"/></robot>')
            with self.assertRaisesRegex(ValueError,'inertia'): csim.load_model(str(p),free_base=True)

if __name__=='__main__': unittest.main()
