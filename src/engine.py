"""骨骼动画IK"""
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
        self._last_sample_time = 0.0

    def add_bone(self, bone: Bone):
        self.bones[bone.name] = bone

    def skin_vertex(self, v: Vertex):
        # 权重归一化，支持任意数量（3/4个）骨骼影响
        total = sum(v.weights.values())
        if total <= 0.0:
            return [0.0, 0.0]
        result = [0.0, 0.0]
        for bname, w in v.weights.items():
            b = self.bones[bname]
            wn = w / total
            result[0] += b.world_pos[0] * wn
            result[1] += b.world_pos[1] * wn
        return result

    def update_world_transforms(self):
        # 按层级从根到叶：父骨骼先变换，子骨骼在父空间下叠加
        order = []
        visited = set()

        def visit(name):
            if name in visited:
                return
            parent = self.bones[name].parent
            if parent and parent in self.bones:
                visit(parent)
            visited.add(name)
            order.append(name)

        for name in self.bones:
            visit(name)

        world_angles = {}
        for name in order:
            bone = self.bones[name]
            if bone.parent and bone.parent in self.bones:
                parent = self.bones[bone.parent]
                parent_angle = world_angles[bone.parent]
                bone.world_pos[0] = parent.world_pos[0] + bone.length * math.cos(parent_angle)
                bone.world_pos[1] = parent.world_pos[1] + bone.length * math.sin(parent_angle)
                world_angles[name] = parent_angle + bone.local_rot
            else:
                world_angles[name] = bone.local_rot
                bone.world_pos = [0.0, 0.0]

    def solve_ik(self, end_effector: str, target: list, max_iter=10, tol=0.01, max_step=0.5):
        # 有最大迭代次数与角度步长限制，收敛或超限都停止
        bone = self.bones[end_effector]
        for _ in range(max_iter):
            dist = math.hypot(target[0]-bone.world_pos[0], target[1]-bone.world_pos[1])
            if dist < tol:
                break
            desired = math.atan2(target[1]-bone.world_pos[1], target[0]-bone.world_pos[0])
            current = math.atan2(bone.world_pos[1], bone.world_pos[0]) \
                if (bone.world_pos[0] or bone.world_pos[1]) else 0.0
            delta = (desired - current + math.pi) % (2 * math.pi) - math.pi
            delta = max(-max_step, min(max_step, delta))
            if abs(delta) < 1e-6:
                break
            bone.local_rot += delta
            self.update_world_transforms()

    def compress_keyframes(self, keyframes, abs_threshold=0.01, rel_threshold=0.01):
        # 相对误差 + 绝对误差双阈值，快速运动不会被过度压缩
        if len(keyframes) <= 2:
            return list(keyframes)
        result = [keyframes[0]]
        for i in range(1, len(keyframes)-1):
            prev = result[-1]
            cur = keyframes[i]
            threshold = max(abs_threshold, rel_threshold * max(abs(prev), abs(cur)))
            if abs(cur - prev) > threshold:
                result.append(keyframes[i])
        result.append(keyframes[-1])
        return result

    def blend_2d(self, x, y, clips):
        x0, x1 = sorted(clips.keys())
        # 边界外参数钳制到 [x0, x1]
        x = min(max(x, x0), x1)
        if x1 == x0:
            return clips[x0]
        t = (x - x0) / (x1 - x0)
        return clips[x0] * (1-t) + clips[x1] * t

    def trigger_event(self, time, events, prev_time=None):
        # 按实际采样时间区间触发，不绑定关键帧时间
        if prev_time is None:
            prev_time = self._last_sample_time
        for ev_time, ev in sorted(events, key=lambda e: e[0]):
            if prev_time < ev_time <= time:
                self.anim_events.append(ev)
        self._last_sample_time = time

    def retarget(self, source_skeleton, target_skeleton):
        # 旋转直接拷贝，位移按骨骼长度比例缩放
        for name in source_skeleton.bones:
            if name in target_skeleton.bones:
                sbone = source_skeleton.bones[name]
                tbone = target_skeleton.bones[name]
                tbone.local_rot = sbone.local_rot
                scale = tbone.length / sbone.length if sbone.length > 0 else 1.0
                tbone.world_pos = [sbone.world_pos[0] * scale,
                                   sbone.world_pos[1] * scale]
