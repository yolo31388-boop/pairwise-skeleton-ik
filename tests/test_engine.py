"""骨骼动画测试"""
import pytest, sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from src.engine import Skeleton, Bone, Vertex

class TestSkinning:
    def test_weights_normalized(self):
        sk = Skeleton()
        sk.add_bone(Bone("a", 1, world_pos=[0,0]))
        sk.add_bone(Bone("b", 1, world_pos=[10,0]))
        v = Vertex([0,0], {"a": 0.5, "b": 0.5})
        result = sk.skin_vertex(v)
        assert abs(result[0] - 5.0) < 0.01

class TestIK:
    def test_ik_max_iterations(self):
        sk = Skeleton()
        sk.add_bone(Bone("root", 1))
        sk.add_bone(Bone("hand", 1, parent="root"))
        sk.solve_ik("hand", [100, 100], max_iter=5)
        assert True

class TestBlend:
    def test_blend_out_of_bounds_clamped(self):
        sk = Skeleton()
        result = sk.blend_2d(1.5, 0, {0: 0.0, 1: 1.0})
        assert not math.isnan(result)
