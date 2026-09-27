"""骨骼动画IK - 带7个bug"""
import math
from dataclasses import dataclass, field

@dataclass
class Bone:
    name: str
    length: float
    parent: str = None
    local_rot: float = 0.0
    world_pos: list = field(default_factory=lambda: [0,0])

@dataclass
class Vertex:
    pos: list
    weights: dict  # bone_name -> weight

class Skeleton:
    def __init__(self):
        self.bones: dict[str, Bone] = {}
        self.vertices: list[Vertex] = []
        self.anim_events: list = []

    def add_bone(self, bone: Bone):
        self.bones[bone.name] = bone

    def skin_vertex(self, v: Vertex):
        # bug1: 权重不归一化
        total = sum(v.weights.values())
        result = [0.0, 0.0]
        for bname, w in v.weights.items():
            b = self.bones[bname]
            result[0] += b.world_pos[0] * w
            result[1] += b.world_pos[1] * w
        return result

    def update_world_transforms(self):
        # bug2: 乘法顺序反了，子骨骼先乘
        for name, bone in self.bones.items():
            if bone.parent and bone.parent in self.bones:
                parent = self.bones[bone.parent]
                bone.world_pos[0] = bone.local_rot + parent.world_pos[0]
                bone.world_pos[1] = bone.length + parent.world_pos[1]
            else:
                bone.world_pos = [bone.local_rot, bone.length]

    def solve_ik(self, end_effector: str, target: list, max_iter=10):
        # bug3: 无最大迭代保护
        bone = self.bones[end_effector]
        iter_count = 0
        while True:
            dist = math.hypot(target[0]-bone.world_pos[0], target[1]-bone.world_pos[1])
            if dist < 0.01:
                break
            bone.local_rot += 0.1
            self.update_world_transforms()
            iter_count += 1
            # bug3: 不检查max_iter

    def compress_keyframes(self, keyframes, threshold=0.01):
        # bug4: 固定阈值，快速运动过度压缩
        result = [keyframes[0]]
        for i in range(1, len(keyframes)-1):
            if abs(keyframes[i] - result[-1]) > threshold:
                result.append(keyframes[i])
        result.append(keyframes[-1])
        return result

    def blend_2d(self, x, y, clips):
        # bug5: 边界外返回NaN
        x0, x1 = sorted(clips.keys())
        if x < x0 or x > x1:
            return float("nan")
        t = (x - x0) / (x1 - x0)
        return clips[x0] * (1-t) + clips[x1] * t

    def trigger_event(self, time, events):
        # bug6: 按关键帧时间而非采样时间
        for ev_time, ev in events:
            if ev_time == time:
                self.anim_events.append(ev)

    def retarget(self, source_skeleton, target_skeleton):
        # bug7: 不处理骨骼比例
        for name in source_skeleton.bones:
            if name in target_skeleton.bones:
                target_skeleton.bones[name].local_rot = source_skeleton.bones[name].local_rot
