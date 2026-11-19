"""Gazebo Harmonic adapter for scheduled static obstacles.

Creation acceptance is followed by scene confirmation. A timed-out mutation is
never retried: the world may have applied it even when its reply was lost.
"""

import math
import time


class GazeboWorld:
    def __init__(self, world, timeout_s=3.0):
        from gz.transport13 import Node

        self.node = Node()
        self.prefix = "/world/" + world
        self.timeout_s = timeout_s

    def scene(self):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene

        ok, response = self.node.request(self.prefix + "/scene/info", Empty(), Empty, Scene, 500)
        if not ok:
            raise RuntimeError("Gazebo scene query timed out")
        return {model.name: model for model in response.model}

    def apply(self, operation, box):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.entity_pb2 import Entity

        before = self.scene()
        if operation.action == "spawn":
            if box.model_name in before:
                raise RuntimeError("Refusing to replace existing model: " + box.model_name)
            request = EntityFactory(sdf=box.sdf(), name=box.model_name, allow_renaming=False)
            service = "/create"
        else:
            if box.model_name not in before:
                raise RuntimeError("Cannot remove absent model: " + box.model_name)
            request = Entity(name=box.model_name, type=Entity.MODEL)
            service = "/remove"
        ok, response = self.node.request(
            self.prefix + service, request, type(request), Boolean, 1500
        )
        if not ok or not response.data:
            raise RuntimeError("Gazebo mutation rejected or outcome unknown: " + operation.id)
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            models = self.scene()
            present = box.model_name in models
            if operation.action == "remove" and not present:
                return dict(entity_name=box.model_name, confirmation="scene_absent")
            if operation.action == "spawn" and present:
                model = models[box.model_name]
                p, q = model.pose.position, model.pose.orientation
                expected = box.pose
                # SceneBroadcaster supplies name/pose here; its is_static field is
                # not populated in Harmonic. Static/collision definitions are bound
                # to the submitted SDF, whose visual and collision extents agree.
                if math.dist((p.x, p.y, p.z), expected[:3]) > 1e-5:
                    raise RuntimeError(
                        f"Created position={(p.x, p.y, p.z)}, expected={expected[:3]}"
                    )
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                if abs(math.atan2(math.sin(yaw - expected[3]), math.cos(yaw - expected[3]))) > 1e-5:
                    raise RuntimeError("Created model orientation differs from scenario")
                return dict(
                    entity_name=box.model_name, entity_id=model.id, confirmation="scene_present"
                )
            time.sleep(0.05)
        raise RuntimeError("Gazebo accepted mutation but scene confirmation timed out")
