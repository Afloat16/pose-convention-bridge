import math
from pathlib import Path
import tempfile
import unittest
from pose_convention_bridge.core import identity, inverse, multiply, rigid, quaternion, convert, similarity, read_poses, output_document


class PoseTests(unittest.TestCase):
    def assertMatrixClose(self,a,b):
        for row_a,row_b in zip(a,b):
            for x,y in zip(row_a,row_b): self.assertAlmostEqual(x,y,places=8)

    def test_inverse_translation_and_rotation(self):
        M=quaternion([math.sqrt(.5),0,0,math.sqrt(.5)],'wxyz'); M[0][3]=1; M[1][3]=2
        self.assertMatrixClose(multiply(inverse(M),M),identity())

    def test_world_to_camera_center_is_not_translation(self):
        M=identity(); M[0][3]=-3
        result=convert([{'matrix':M}], 'w2c','opencv',output_axes='opencv')
        self.assertEqual(result[0]['matrix'][0][3],3)

    def test_basis_flips_camera_axes_not_world_center(self):
        M=identity(); M[0][3]=1; M[1][3]=2; M[2][3]=3
        converted=convert([{'matrix':M}],'c2w','opencv')
        self.assertEqual([row[3] for row in converted[0]['matrix'][:3]],[1,2,3])
        self.assertEqual(converted[0]['matrix'][1][1],-1)
        back=convert(converted,'c2w','opengl',output_axes='opencv')
        self.assertMatrixClose(back[0]['matrix'],M)

    def test_rotated_roundtrip_both_directions(self):
        M=quaternion([.5,.5,.5,.5],'wxyz'); M[0][3]=2
        out=convert([{'matrix':M}],'w2c','opencv','c2w','opengl')
        back=convert(out,'c2w','opengl','w2c','opencv')
        self.assertMatrixClose(back[0]['matrix'],M)

    def test_world_similarity_scales_centers_not_rotation(self):
        M=identity(); M[0][3]=3
        W=identity(); W[0][3]=5
        out=convert([{'matrix':M}],'c2w','opencv',output_axes='opencv',world=W,scale=2)
        self.assertEqual(out[0]['matrix'][0][3],11)
        self.assertEqual(out[0]['matrix'][0][0],1)

    def test_dataparser_translation_is_scaled(self):
        W=identity(); W[0][3]=5
        world,scale=similarity({'transform':W[:3],'scale':2},True)
        M=identity(); M[0][3]=3
        out=convert([{'matrix':M}],'c2w','opencv',output_axes='opencv',world=world,scale=scale)
        self.assertEqual(out[0]['matrix'][0][3],16)

    def test_reflection_scale_and_nan_rejected(self):
        for value in [-1,2,float('nan')]:
            M=identity(); M[0][0]=value
            with self.assertRaises(ValueError): rigid(M)

    def test_tum_quaternion_order_and_no_invented_filename(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'trajectory.txt'
            path.write_text('1000000000.000000001 1 2 3 0 0 0 1\n')
            frames,_=read_poses(path,'tum')
            self.assertEqual(frames[0]['timestamp'],'1000000000.000000001')
            self.assertNotIn('file_path',frames[0])
            mapped,_=read_poses(path,'tum',{'1000000000.000000001':'real.png'})
            self.assertEqual(mapped[0]['file_path'],'real.png')

    def test_colmap_names_and_wxyz(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'images.txt'; path.write_text('10 1 0 0 0 -3 0 0 7 name with spaces.png\n\n')
            frames,_=read_poses(path,'colmap-images')
            self.assertEqual(frames[0]['file_path'],'name with spaces.png')
            out=convert(frames,'w2c','opencv',output_axes='opencv')
            self.assertEqual(out[0]['matrix'][0][3],3)

    def test_nerfstudio_requires_real_intrinsics(self):
        frames=[{'id':'1','file_path':'a.png','matrix':identity()}]
        with self.assertRaises(ValueError): output_document(frames,'c2w','opengl','nerfstudio')
        metadata={'fl_x':500,'fl_y':500,'cx':320,'cy':240,'w':640,'h':480}
        self.assertIn('transform_matrix',output_document(frames,'c2w','opengl','nerfstudio',metadata)['frames'][0])
        with self.assertRaises(ValueError): output_document(frames,'w2c','opencv','nerfstudio',metadata)
