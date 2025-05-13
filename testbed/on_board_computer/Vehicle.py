from shapely.geometry import Polygon
from shapely.geometry import Point, box
from pyproj import CRS, Transformer
from shapely.ops import transform
from shapely.affinity import rotate
import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
import json
import datetime
import time
import threading
from Vehicle_subscriber import Vehicle_subscriber

class Vehicle:
    def __init__(self, veh_id):
        self.veh_id = veh_id       
        self.physical_mqtt_server = None
        self.physical_mqtt_port = None
        self.virtual_mqtt_server = None
        self.virtual_mqtt_port = None
        self.subscribe_status = False
        self._last_publish_step = 0
        self._geofenceInfo = {}
        self._physicalComputerMapping = None

    '''Note: pub_client與broker保持連線目前是透過keepalive參數，因此on_connect與on_disconnect callback不會被調用'''
    def connect(self,physical_mqtt_server, physical_mqtt_port,virtual_mqtt_server, virtual_mqtt_port):
        self.physical_mqtt_server = physical_mqtt_server
        self.physical_mqtt_port = physical_mqtt_port
        self.virtual_mqtt_server = virtual_mqtt_server
        self.virtual_mqtt_port = virtual_mqtt_port
        self.pub_client = mqtt.Client(
            client_id=self.veh_id, protocol=mqtt.MQTTv5)
        self.pub_client._connect_timeout = 60.0    
        self.pub_client.on_connect = self.on_connect
        self.pub_client.on_disconnect = self.on_disconnect        
        print(f"{self.veh_id} connecting to broker {physical_mqtt_server}")
        self.pub_client.connect(physical_mqtt_server, physical_mqtt_port, keepalive = 3600)
        return 0

    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(reason_code)
        if reason_code == 0:
            print(f"{self.veh_id} Connection: {reason_code}")
        else:
            print("Connection failed, reason code: ", reason_code)

    def disconnect(self):
        self.pub_client.disconnect()
        if self.vehicle_subscriber:
            self.vehicle_subscriber.disconnect()

    def on_disconnect(self, client, userdata, reason_code, properties):
        if reason_code == 0:
            print(f"{self.veh_id} disonnected:", reason_code)
            # self.pub_client.loop_stop()
        else:
            print(f"{self.veh_id} disonnected error:", reason_code)
    
    @property
    def physicalComputerMapping(self):
        return self._physicalComputerMapping

    @physicalComputerMapping.setter
    def physicalComputerMapping(self, pc):
        self._physicalComputerMapping = pc

    @property
    def last_publish_step(self):
        return self._last_publish_step

    @last_publish_step.setter
    def last_publish_step(self, last_publish_step):
        self._last_publish_step = last_publish_step 

    @property
    def geofenceInfo(self):
        return self._geofenceInfo

    @geofenceInfo.setter
    def geofenceInfo(self, geofenceInfo):
        self._geofenceInfo = geofenceInfo    

    def publishGeoFence(self):      
        geofenceInfo = self.geofenceInfo
        roatateAngle = (geofenceInfo['laneAngle'] - 90) * -1 # clock-wise rotation
        geofence = self.rectangle_geodesic_point_buffer(lat=geofenceInfo['lat'], lon=geofenceInfo['lon'], width=geofenceInfo['width'], roatateAngle=roatateAngle)
        
        vehicle_position = tuple([geofenceInfo['lat'], geofenceInfo['lon']])
        currentTime = datetime.datetime.now()        
        publish_properties = Properties(PacketTypes.PUBLISH)
        publish_properties.UserProperty = ("geofence", str(geofence))
        publish_properties.UserProperty = ("time", str(currentTime))
        publish_properties.UserProperty = ("laneID", str(geofenceInfo['laneID']))
        publish_properties.UserProperty = ("speed", str(geofenceInfo['speed']))
        publish_properties.UserProperty = ("laneLength", str(geofenceInfo['width'] * 3))
        publish_properties.UserProperty = ("travelTime", str(geofenceInfo['travelTime']))
        publish_properties.UserProperty = ("lanePosition", str(geofenceInfo['lanePosition']))
        publish_properties.UserProperty = ("connectedLanes", str(geofenceInfo['connectedLanes']))         
        
        payload_dict = dict(veh_id = self.veh_id, lat=geofenceInfo['lat'], lon=geofenceInfo['lon'])
        publish_payload = json.dumps(payload_dict)        
               
        self.pub_client.publish(
            topic="geofence", payload=publish_payload, properties=publish_properties)
     
        if not self.subscribe_status:
            subscriber_id = f'{self.veh_id}_subscriber'
            vehicleLength = float(geofenceInfo['vehicleLength'])
            vehicle_subscriber = Vehicle_subscriber(subscriber_id,geofenceInfo['vehicleLength'], self.virtual_mqtt_server, self.virtual_mqtt_port)
            vehicle_subscriber.connect(self.physical_mqtt_server,self.physical_mqtt_port)
            self.vehicle_subscriber = vehicle_subscriber                  
            self.subscribe_status = True

    ''' 計算圓形的地理圍欄 '''
    def circle_geodesic_point_buffer(self, lat, lon, radius):
        # Azimuthal equidistant projection
        aeqd_proj = CRS.from_proj4(
            f"+proj=aeqd +lat_0={lat} +lon_0={lon} +x_0=0 +y_0=0")
        tfmr = Transformer.from_proj(aeqd_proj, aeqd_proj.geodetic_crs)
        buf = Point(0, 0).buffer(radius)  # distance in metres
        return transform(tfmr.transform, buf).exterior.coords[:]

    ''' 計算矩形的地理圍欄'''
    def rectangle_geodesic_point_buffer(self, lat, lon, width, roatateAngle, height=15):
        # Azimuthal equidistant projection
        aeqd_proj = CRS.from_proj4(
            f"+proj=aeqd +lat_0={lat} +lon_0={lon} +x_0=0 +y_0=0")
        tfmr = Transformer.from_proj(aeqd_proj.geodetic_crs, aeqd_proj)
        projected_point = transform(tfmr.transform, Point(lon, lat))
        # Create a rectangle centered at the projected point
        minx = projected_point.x - (width / 2)
        maxx = projected_point.x + (width / 2)
        miny = projected_point.y - (height / 2)
        maxy = projected_point.y + (height / 2)
        projected_rect = box(minx, miny, maxx, maxy)
        # Rotate the rectangle by the specified angle        
        rotated_rect = rotate(projected_rect, roatateAngle, origin='center', use_radians=False)
        # Transform the rotated rectangle back to geodetic coordinates
        tfmr_inv = Transformer.from_proj(aeqd_proj, aeqd_proj.geodetic_crs)
        geodetic_rect = transform(
            tfmr_inv.transform, rotated_rect).exterior.coords[:]
        return geodetic_rect
    
    def publish(self):
        self.pub_client.publish(
            topic="keepAlive", payload="i")
