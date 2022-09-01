import paho.mqtt.client as mqtt
import json
import time
import math
import random

MQTT_NAME = "avp_infra"
MQTT_PWD = "123456"
MQTT_HOST = "127.0.0.1"
# MQTT_HOST = "iot.dfrobot.com.cn"
# MQTT_NAME = "3BM8z3h7R"
# MQTT_PWD = "qfGUz3hngz"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "avp_infra_ID"

SIM_TIME_STEP = 0.1  # second

MAP_MIN_X = -100
MAP_MAX_X = 100
MAP_MIN_Y = -100
MAP_MAX_Y = 100


class SimVehicle():
    def __init__(self, id, x, y, yaw):
        self.id = id
        self.status = 0  # 状态位（后续会扩展），类型：uint8_t，0：Unknown，1:Normal, 2:Invalid
        self.x = x  # meter
        self.y = y  # meter
        self.yaw = yaw  # degree, 0~360
        self.steer = 0  # degree, -90~90
        self.speed = 0
        self.l = 4.0
        self.w = 1.8
        self.rz = 0.0
        self.wb = 3.0
        self.LtoRear = 0.8
        self.dt = SIM_TIME_STEP

    def set(self, steer, speed):
        self.steer = steer
        self.speed = speed

    def move(self):
        self.x += self.speed * math.cos(math.radians(self.yaw)) * self.dt
        self.y += self.speed * math.sin(math.radians(self.yaw)) * self.dt
        self.yaw += math.degrees(self.speed / self.wb *
                                 math.tan(math.radians(self.steer)) * self.dt)
        self.yaw %= 360

        if self.x < MAP_MIN_X or self.x > MAP_MAX_X or self.y < MAP_MIN_Y or self.y > MAP_MAX_Y:
            self.x = 0.0
            self.y = 0.0

    def toDict(self):
        return dict({
            "id": self.id,
            "status": self.status,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "speed": self.speed,
            "steer": self.steer,
            "l": self.l,
            "w": self.w,
            "rz": self.rz,
            "wb": self.wb,
            "LtoRear": self.LtoRear,
        })


def GenerateRandomPolygon():
    OBSTACLE_RADIUS_MIN = 1.5
    OBSTACLE_RADIUS_MAX = 2.0
    obs_center_x = random.randint(MAP_MIN_X, MAP_MAX_X)
    obs_center_y = random.randint(MAP_MIN_Y, MAP_MAX_Y)
    # print(obs_center_x, obs_center_y)
    ret_polygon = []
    n_obs_vertex = random.randint(3, 6)  # [3, 7)
    amplify = random.uniform(1.0, 3.0)
    angles = [random.uniform(0, 2 * math.pi) for i in range(n_obs_vertex)]
    angles.sort()  # arrange from small to large, prevent crossing
    radius = [
        random.uniform(OBSTACLE_RADIUS_MIN * amplify,
                       OBSTACLE_RADIUS_MAX * amplify)
        for i in range(n_obs_vertex)
    ]
    for j in range(n_obs_vertex):
        obs_vertex_x = obs_center_x + radius[j] * math.cos(angles[j])
        obs_vertex_y = obs_center_y + radius[j] * math.sin(angles[j])
        ret_polygon.append([obs_vertex_x, obs_vertex_y])
    return ret_polygon


class Obstacle():
    # @param polygon [[x1, y1], [x2, y2], ..., [xn, yn]]
    def __init__(self, id, polygon, motion=1) -> None:
        self.id = id
        self.type = 1  # 类型（后续会扩展），类型：uint8_t，0:Unknown，1:Vehicle, 2:Pedestrian，3:Cone,
        self.motion = motion  #  动静态目标（后续会扩展），类型：uint8_t 0:Unknown，1:Static, 2:Moving
        self.polygon = polygon
        sum_x = 0.0
        sum_y = 0.0
        for point in polygon:
            sum_x += point[0]
            sum_y += point[1]
        self.x = sum_x / len(polygon)
        self.y = sum_y / len(polygon)
        if (self.motion == 2):
            self.vx = 0.1
            self.vy = 0.1
        else:
            self.vx = 0.0
            self.vy = 0.0

    def move(self) -> None:
        if self.motion != 2:
            return
        self.x += self.vx
        self.y += self.vy
        for point in self.polygon:
            point[0] += self.vx
            point[1] += self.vy

    def toDict(self):
        return dict({
            "id": self.id,
            "type": self.type,
            "motion": self.motion,
            "polygon": self.polygon,
            "x": self.x,
            "y": self.y,
            "vx": self.vx,
            "vy": self.vy,
        })


class MQTTClient():
    def __init__(self):
        self._pub_seq = 0
        self._global_seq = 0
        self._ptc_cb_seq = 0
        self._ptc_mqttpub_seq = 0
        self._ego_cb_seq = 0
        self._ego_mqttpub_seq = 0
        self._exit_flag = False
        self._vehicle_dict = dict()
        self._obstacle_list = list()
        # self.tk_root = None

        # Initialize mqtt client
        self._mqtt = mqtt.Client(client_id=MQTT_CLIENT_ID)
        self._mqtt.username_pw_set(MQTT_NAME, MQTT_PWD)
        self._mqtt.on_connect = self.cb_connected
        self._mqtt.on_disconnect = self.cb_disconnect

        ret = self._mqtt.connect(MQTT_HOST, MQTT_PORT, 60)
        if ret == 0:
            print("MQTT initialize success")
        else:
            print("MQTT initialize failed!")
            exit(0)

    def cb_connected(self, a, b, c, d):
        print("MQTT connected")

    def cb_disconnect(self, a, b, c):
        print("MQTT disconnected")
        self._exit_flag = True

    def setup_vehicles(self):
        # for i in range(0, 10):
        #     steer = 30 * random.randint(0, 100) / 100.0  # degree
        #     speed = 5 * random.randint(0, 100) / 100.0  #m/s
        #     vehicle = SimVehicle(i, -500 + 50 * i, 0, i * 10)
        #     vehicle.set(steer, speed)
        #     self._vehicle_dict[i] = vehicle

        sv1 = SimVehicle(1, 0, 0, -90)
        self._vehicle_dict[sv1.id] = sv1
        sv2 = SimVehicle(12, 10, 10, math.pi / 3)
        self._vehicle_dict[sv2.id] = sv2
        # sv3 = SimVehicle(13, 18, 15, -math.pi / 2)
        # self._vehicle_dict[sv3.id] = sv3
        # sv4 = SimVehicle(14, -25, -15, math.pi)
        # self._vehicle_dict[sv4.id] = sv4
        # sv5 = SimVehicle(20, 10, 0, 0)
        # self._vehicle_dict[sv5.id] = sv5
        # sv6 = SimVehicle(30, 10, 0, 0)
        # self._vehicle_dict[sv6.id] = sv6

    def stop_running(self):
        print("Exit by button!")
        self._exit_flag = True

    def run(self):
        # self.setup_subscriber()
        self._mqtt.loop_start()
        payload = {
            "name": self._mqtt._username.decode("utf-8"),
            "seq": 0,
            "time": 0.0,
            "payload": "beat"
        }

        counter = 0
        beat_seq = 0
        vehicle_seq = 0
        obstacle_seq = 0
        obstacle_count = 1

        # Set up vehicles
        self.setup_vehicles()

        # Set up obstacles
        for id in range(obstacle_count):
            obstacle = Obstacle(id=id, polygon=GenerateRandomPolygon())
            self._obstacle_list.append(obstacle)
        self._long_lasting_obstacle = Obstacle(id=obstacle_count,
                                               polygon=[
                                                   [0., 0.],
                                                   [2., 0.],
                                                   [2., 2.],
                                                   [0., 2.],
                                               ],
                                               motion=2)
        obstacle_count += 1

        # self.setup_tk()

        while self._exit_flag is False:
            # send beat/avp_infra
            if counter % 50 == 0:
                beat_seq = beat_seq + 1
                payload["seq"] = beat_seq
                payload["time"] = time.time()
                payload["payload"] = "beat"
                topic = "beat/avp_infra"
                self._mqtt.publish(topic, json.dumps(payload), 0)
                # print("[Tx][TOPIC: " + topic + "] MSG: " + json.dumps(payload))

            # send perception/vehicles
            if counter % 10 == 0:
                vehicle_seq = vehicle_seq + 1
                payload_list = list()
                for v in self._vehicle_dict.values():
                    v.move()
                    payload_list.append(v.toDict())

                payload["seq"] = vehicle_seq
                payload["time"] = time.time()
                payload["sender"] = "IPS"
                payload["payload"] = payload_list
                topic = "perception/vehicles"
                self._mqtt.publish(topic, json.dumps(payload), 0)
                print("[Tx][TOPIC: " + topic + "] MSG: " + json.dumps(payload))

            # send perception/obstacles
            if counter % 10 == 0:
                obstacle_seq = obstacle_seq + 1
                payload_list = list()
                for obstacle in self._obstacle_list:
                    payload_list.append(obstacle.toDict())
                self._long_lasting_obstacle.move()
                payload_list.append(self._long_lasting_obstacle.toDict())
                payload["seq"] = obstacle_seq
                payload["time"] = time.time()
                payload["sender"] = "IPS"
                payload["payload"] = payload_list
                topic = "perception/obstacles"
                self._mqtt.publish(topic, json.dumps(payload), 0)
                # print("[Tx][TOPIC: " + topic + "] MSG: " + json.dumps(payload))

            # update obstacles
            if counter % 100 == 0:
                del self._obstacle_list[0]
                obstacle = Obstacle(id=obstacle_count,
                                    polygon=GenerateRandomPolygon())
                self._obstacle_list.append(obstacle)
                obstacle_count += 1

            # self.tk_root.update_idletasks()
            # self.tk_root.update()

            counter = counter + 1
            time.sleep(0.02)

if __name__ == '__main__':
    mqtt_client_node = MQTTClient()
    mqtt_client_node.run()