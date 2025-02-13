from comms.vision.proto_compiled import *
import socket, threading, time
import numpy as np
from constants import *
from world.world import World
from sysmic_kit import *

ListPackets = list[SSL_WrapperPacket]

class Vision:
    """ Deserializa paquetes de visión con Filtro de Kalman """
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, multi_cast_address, port_ssl, world: World):
        self.world = world
        self.ball: SSL_DetectionBall = SSL_DetectionBall()
        
        # Crear socket UDP
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socket.bind(('', port_ssl))
        
        # Unirse al grupo multicast
        mreq = socket.inet_aton(multi_cast_address) + socket.inet_aton('0.0.0.0')
        self.udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        self.last_packet_time = time.time()
        self.time_between_packet = 0
        
        # Diccionarios para almacenar los filtros de Kalman por cada objeto
        self.kalman_robots = {}
        self.kalman_ball = self._init_kalman_filter()
    
    def _init_kalman_filter(self):
        """Inicializa el filtro de Kalman"""
        dt = 1/60
        #matriz de transicion de estado
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        # Matriz observacion, descarta velocidad al comparar con datos reales del sensor
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])
        #valor de incertidumbre: A mayor P menos se confia en el modelo
        P = np.eye(4) * 300
        # Confianza en la prediccion: a mayor Q el filtro permite cambios bruscos, a menor Q el filtro suaviza los movimientos (si estos son mas predecibles)
        Q = np.eye(4) * 1e-3
        #Confianza en datos del sensor: Aumentar R si se desconfia del sensor
        R = np.eye(2) * 1e-1
        return {"F": F, "H": H, "P": P, "Q": Q, "R": R, "x": np.zeros((4, 1))}
    
    def _kalman_predict(self, kf):
        kf["x"] = np.dot(kf["F"], kf["x"])
        kf["P"] = np.dot(np.dot(kf["F"], kf["P"]), kf["F"].T) + kf["Q"]
    
    def _kalman_update(self, kf, measurement):
        z = np.array(measurement).reshape(2, 1) # medicion real
        y = z - np.dot(kf["H"], kf["x"]) #error
        S = np.dot(kf["H"], np.dot(kf["P"], kf["H"].T)) + kf["R"] #incertidumbre total
        K = np.dot(np.dot(kf["P"], kf["H"].T), np.linalg.inv(S)) #ganancia de kalman
        kf["x"] += np.dot(K, y)
        I = np.eye(kf["P"].shape[0])
        kf["P"] = np.dot(I - np.dot(K, kf["H"]), kf["P"])
        return kf["x"][:2].flatten()
    
    def loop(self):
        while True:
            self._receive_vision_packets()
            time.sleep(FRAME_RATE)
    
    def _receive_vision_packets(self):
        packets: ListPackets = []
        current_time = time.time()
        
        try:
            while True:
                self.udp_socket.settimeout(0.001)
                data, _ = self.udp_socket.recvfrom(4096)
                packet = SSL_WrapperPacket()
                if not packet.ParseFromString(data):
                    print('Error in _receive_vision_packets: cannot parse packet')
                else:
                    packets.append(packet)
        except socket.timeout:
            pass
        
        self.time_between_packet = (current_time - self.last_packet_time)
        if packets:
            self.last_packet_time = current_time
            self._update(packets)
    
    def _update(self, packets: ListPackets):
        robots_blue = {}
        robots_yellow = {}
        
        for packet in packets:
            det = packet.detection
            for ball in det.balls:
                self.ball = ball
            for robot_data in det.robots_blue:
                robots_blue[robot_data.robot_id] = robot_data
            for robot_data in det.robots_yellow:
                robots_yellow[robot_data.robot_id] = robot_data
        
        self._update_world(robots_blue.values(), robots_yellow.values(), self.ball)
    
    def _update_world(self, blue, yellow, ball):
        for robot in blue:
            if robot.robot_id not in self.kalman_robots:
                self.kalman_robots[robot.robot_id] = self._init_kalman_filter()
            kalman = self.kalman_robots[robot.robot_id]
            self._kalman_predict(kalman)
            filtered_pos = self._kalman_update(kalman, [robot.x / 1000, robot.y / 1000])
            data = RobotData(robot.robot_id, TeamColor.BLUE)
            data.position = Vector2(*filtered_pos)
            data.orientation = robot.orientation
            data.last_time_update = time.time()
            self.world._vision_robot_update(data)
        
        for robot in yellow:
            if robot.robot_id not in self.kalman_robots:
                self.kalman_robots[robot.robot_id] = self._init_kalman_filter()
            kalman = self.kalman_robots[robot.robot_id]
            self._kalman_predict(kalman)
            filtered_pos = self._kalman_update(kalman, [robot.x / 1000, robot.y / 1000])
            data = RobotData(robot.robot_id, TeamColor.YELLOW)
            data.position = Vector2(*filtered_pos)
            data.orientation = robot.orientation
            data.last_time_update = time.time()
            self.world._vision_robot_update(data)
        
        self._kalman_predict(self.kalman_ball)
        filtered_ball_pos = self._kalman_update(self.kalman_ball, [ball.x / 1000, ball.y / 1000])
        self.world._vision_ball_update(Vector2(*filtered_ball_pos))

#https://github.com/mbshbn/Kalman-filter-for-robotics
