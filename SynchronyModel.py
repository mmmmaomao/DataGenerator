import queue
import sys
import random
import logging

import numpy as np

from config import config_to_trans
from data_utils import camera_intrinsic, filter_by_distance

sys.path.append("/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg")

import carla


class SynchronyModel:
    def __init__(self, cfg):
        self.cfg = cfg
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.traffic_manager = self.client.get_trafficmanager()
        self.init_settings = None
        self.frame = None
        self.actors = {"non_agents": [], "walkers": [], "agents": [], "sensors": {}}
        self.data = {"sensor_data": {}, "environment_data": None}  # 记录每一帧的数据
        self.vehicle = None

    def set_synchrony(self):
        self.init_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

    def setting_recover(self):
        for agent in self.actors["agents"]:
            for sensor in self.actors["sensors"][agent]:
                sensor.destroy()
            agent.destroy()
        batch = []
        for actor_id in self.actors["non_agents"]:
            batch.append(carla.command.DestroyActor(actor_id))
        for walker_id in self.actors["walkers"]:
            batch.append(carla.command.DestroyActor(walker_id))
        self.client.apply_batch_sync(batch)
        self.world.apply_settings(self.init_settings)

    def spawn_actors(self):
        num_of_vehicles = self.cfg["CARLA_CONFIG"]["NUM_OF_VEHICLES"]
        num_of_walkers = self.cfg["CARLA_CONFIG"]["NUM_OF_WALKERS"]

        # 生成车辆actors
        blueprints = self.world.get_blueprint_library().filter("vehicle.*")
        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if num_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
            num_of_vehicles = num_of_vehicles
        elif num_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, num_of_vehicles, number_of_spawn_points)
            num_of_vehicles = number_of_spawn_points

        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= num_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(carla.command.SpawnActor(blueprint, transform))

            for response in self.client.apply_batch_sync(batch):
                if response.error:
                    continue
                else:
                    self.actors["non_agents"].append(response.actor_id)

        # 生成行人actors
        blueprintsWalkers = self.world.get_blueprint_library().filter("walker.pedestrian.*")
        spawn_points = []
        for i in range(num_of_walkers):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        batch = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

        for response in self.client.apply_batch_sync(batch, True):
            if response.error:
                continue
            else:
                self.actors["walkers"].append(response.actor_id)
        print("spawn {} vehicles and {} walkers".format(len(self.actors["non_agents"]),
                                                        len(self.actors["walkers"])))
        self.world.tick()

    def set_actors_route(self):
        self.traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        self.traffic_manager.set_synchronous_mode(True)
        vehicle_actors = self.world.get_actors(self.actors["non_agents"])
        for vehicle in vehicle_actors:
            vehicle.set_autopilot(True, self.traffic_manager.get_port())

        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        batch = []
        for i in range(len(self.actors["walkers"])):
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(),
                                                  self.actors["walkers"][i]))
        controllers_id = []
        for response in self.client.apply_batch_sync(batch, True):
            if response.error:
                pass
            else:
                controllers_id.append(response.actor_id)
        self.world.set_pedestrians_cross_factor(0.2)

        for con_id in controllers_id:
            # start walker
            self.world.get_actor(con_id).start()
            # set walk to random point
            destination = self.world.get_random_location_from_navigation()
            self.world.get_actor(con_id).go_to_location(destination)
            # max speed
            self.world.get_actor(con_id).set_max_speed(10)

    def spawn_agent(self):
        vehicle_bp = random.choice(self.world.get_blueprint_library().filter(self.cfg["AGENT_CONFIG"]["BLUEPRINT"]))
        trans_cfg = self.cfg["AGENT_CONFIG"]["TRANSFORM"]
        transform = config_to_trans(trans_cfg)
        transform = random.choice(self.world.get_map().get_spawn_points())
        agent = self.world.spawn_actor(vehicle_bp, transform)
        agent.set_autopilot(True, self.traffic_manager.get_port())
        self.actors["agents"].append(agent)

        self.actors["sensors"][agent] = []
        for sensor, config in self.cfg["SENSOR_CONFIG"].items():
            sensor_bp = self.world.get_blueprint_library().find(config["BLUEPRINT"])
            for attr, val in config["ATTRIBUTE"].items():
                sensor_bp.set_attribute(attr, str(val))
            trans_cfg = config["TRANSFORM"]
            transform = carla.Transform(carla.Location(trans_cfg["location"][0],
                                                       trans_cfg["location"][1],
                                                       trans_cfg["location"][2]),
                                        carla.Rotation(trans_cfg["rotation"][0],
                                                       trans_cfg["rotation"][1],
                                                       trans_cfg["rotation"][2]))
            sensor = self.world.spawn_actor(sensor_bp, transform, attach_to=agent)
            self.actors["sensors"][agent].append(sensor)
        self.world.tick()

    def sensor_listen(self):
        for agent, sensors in self.actors["sensors"].items():
            self.data["sensor_data"][agent] = []
            for sensor in sensors:
                q = queue.Queue()
                self.data["sensor_data"][agent].append(q)
                sensor.listen(q.put)

    def tick(self):
        ret = {"environment_objects": None, "actors": None, "agents_data": {}}
        self.frame = self.world.tick()

        ret["environment_objects"] = self.world.get_environment_objects(carla.CityObjectLabel.Any)
        ret["actors"] = self.world.get_actors()
        image_width = self.cfg["SENSOR_CONFIG"]["RGB"]["ATTRIBUTE"]["image_size_x"]
        image_height = self.cfg["SENSOR_CONFIG"]["RGB"]["ATTRIBUTE"]["image_size_y"]
        for agent, dataQue in self.data["sensor_data"].items():
            data = [self._retrieve_data(q) for q in dataQue]
            assert all(x.frame == self.frame for x in data)
            ret["agents_data"][agent] = {}
            ret["agents_data"][agent]["sensor_data"] = data
            ret["agents_data"][agent]["intrinsic"] = camera_intrinsic(image_width, image_height)
            ret["agents_data"][agent]["extrinsic"] = np.mat(
                self.actors["sensors"][agent][0].get_transform().get_matrix())
        filter_by_distance(ret, self.cfg["FILTER_CONFIG"]["PRELIMINARY_FILTER_DISTANCE"])
        return ret

    def _retrieve_data(self, q):
        while True:
            data = q.get()
            if data.frame == self.frame:
                return data
