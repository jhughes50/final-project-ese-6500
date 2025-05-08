"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import os

import folium
from flask import Flask, render_template_string, render_template
from flask_socketio import SocketIO
from threading import Thread

from typing import Tuple

class MapApp:

    def __init__(self, path : str, ip : str = '127.0.0.1', port : int = 5000, starting_ll: Tuple[float] = (39.94136, -75.199492)) -> None:
        print("[VISUALIZER] Starting viz on ip: %s:%i" %(ip, port))
        self.ip_ = ip
        self.port_ = port 

        self.app_ = Flask(__name__, template_folder=path+'/templates', static_folder=path+'/static')
        self.socketio_ = SocketIO(self.app_, cors_allowed_origins="*")
        
        self.map_ = folium.Map(location=starting_ll,
                               zoom_start=24,
                               tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                               attr='Esri',
                               name='Esri Satellite')


        self.setup_routes()
        self.gps_points = list()
        self.odom_points = list()

    def setup_routes(self) -> None:
        @self.app_.route('/')
        def index():
            return render_template('index.html')

    def update_gps(self, lat: float, lon: float, popup: str = "gps"):
        """Update map with new point and source type"""
        point_data = {
            "lat": lat,
            "lon": lon,
            "popup": popup}
        
        self.gps_points.append(point_data)
            
        # Send all points to frontend
        self.socketio_.emit('gps_update', {"points": self.gps_points})

    def update_odom(self, lat: float, lon: float, popup: str = "odom") -> None:
        point_data = {"lat": lat, "lon": lon, "popup": popup}

        self.odom_points.append(point_data)

        self.socketio_.emit('odom_update', {"points": self.odom_points})


    def run_in_thread(self) -> None:
        thread = Thread(target=self.run)
        thread.daemon = True
        thread.start()

    def run(self) -> None:
        self.app_.run(debug=False, host=self.ip_, port=self.port_)

