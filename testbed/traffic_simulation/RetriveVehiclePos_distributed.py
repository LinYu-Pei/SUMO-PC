import os
import sys
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import time
from Vehicle import Vehicle
from Vehicle_subscriber import Vehicle_subscriber
from garbage_collector import garbage_collector
import threading
from Vehicle_dispatcher import Vehicle_dispatcher


def retrieve_SUMO_vehicle_info(traci, veh_id):
    x, y = traci.vehicle.getPosition(veh_id)           
    lon, lat = traci.simulation.convertGeo(x, y)
    laneID = traci.vehicle.getLaneID(veh_id)
    vehicleLength = traci.vehicle.getLength(veh_id)
    lanePosition = traci.vehicle.getLanePosition(veh_id)             
    speed = traci.vehicle.getSpeed(veh_id) * 3.6  # conversion from [m/s] to [km/hr]

    laneAngle = traci.lane.getAngle(laneID)            
    laneLength = traci.lane.getLength(laneID)
    travelTime= traci.lane.getTraveltime(laneID)

    # if junction, get connected lanes
    connectedLanes = []
    if laneID.startswith(":"):            
        lane_links = traci.lane.getLinks(laneID,False)
        link_number = traci.lane.getLinkNumber(laneID)
        for connectedLane in lane_links:
            connectedLanes.append(connectedLane[0])
            
    geofenceInfo = dict(lat=lat, lon=lon, laneID=laneID, width=laneLength/3, laneAngle=laneAngle, speed=speed, travelTime=travelTime, lanePosition=lanePosition, vehicleLength=vehicleLength, connectedLanes=connectedLanes)
    
    return geofenceInfo

if __name__ == '__main__':
    Vehicle_dispatcher = Vehicle_dispatcher()    
    computers = dict(pc1='127.0.0.1')
    Vehicle_dispatcher.physicalComputers = computers
    computer_count = len(computers)
    pc_list = list(Vehicle_dispatcher.physicalComputers.keys())
    Vehicle_dispatcher.connect('localhost', 7884)

    sumoBinary = "/home/tim/sumo/bin/sumo-gui"
    sumocfg = "/home/tim/traffic_simulation/osm.sumocfg"
    sumoCmd = [sumoBinary, "-c", sumocfg, "-d", "1000", "-S"]
    traci.start(sumoCmd)
    print("Simulation Start!")

    vehicleDict = dict()
    current_simulation_step = 0    
    steps={}
    event_occur_steps = set()
    geofenceInfos = {}  
    publish_period = 5  
    
    i = 0
    while traci.simulation.getMinExpectedNumber() > 0:        

        garbage_collector(physical_mqtt_server='127.0.0.1', physical_mqtt_port=7883, virtual_mqtt_server='127.0.0.1', virtual_mqtt_port=7884, traci_simulation=traci.simulation, vehicleDict=vehicleDict)        

        for veh_id in traci.vehicle.getIDList():
            if veh_id in vehicleDict:
                vehicle = vehicleDict[veh_id]
            else:
                vehicle = Vehicle(veh_id)
                vehicleDict[veh_id] = vehicle                

            pc_index = i % computer_count
            i+=1
            if vehicle.physicalComputerMapping == None:
                vehicle.physicalComputerMapping = pc_list[pc_index]                               

            if vehicle.last_publish_step == 0:
                vehicle.last_publish_step = current_simulation_step
                if steps.get(current_simulation_step) == None:
                    veh_list = []
                    veh_list.append(veh_id)
                    steps[current_simulation_step] = veh_list
                else:
                    veh_list = steps[current_simulation_step]
                    veh_list.append(veh_id)
                    steps[current_simulation_step] = veh_list

                event_occur_steps.add(current_simulation_step)
                info_list = []
                geofenceInfo = retrieve_SUMO_vehicle_info(traci, veh_id)
                info_list.append(geofenceInfo)
                geofenceInfos[veh_id] = info_list
            else:
                if (vehicle.last_publish_step + publish_period) == current_simulation_step:
                    vehicle.last_publish_step = current_simulation_step
                    if steps.get(current_simulation_step) == None:
                        veh_list = []
                        veh_list.append(veh_id)
                        steps[current_simulation_step] = veh_list
                    else:
                        veh_list = steps[current_simulation_step]
                        veh_list.append(veh_id)
                        steps[current_simulation_step] = veh_list

                    event_occur_steps.add(current_simulation_step)
                    info_list = geofenceInfos[veh_id]
                    geofenceInfo = retrieve_SUMO_vehicle_info(traci, veh_id)
                    info_list.append(geofenceInfo)
        
        # dispatch_start = time.monotonic()
        veh_count = 0         
        for step in event_occur_steps:               
            veh_list = steps[step]
            veh_count = len(veh_list)
            # publish_start = time.monotonic()
            for veh_id in veh_list:
                infos = geofenceInfos.get(veh_id)                    
                info = infos.pop()
                vehicle = vehicleDict[veh_id]
                pc = vehicle.physicalComputerMapping
                Vehicle_dispatcher.dispatch_vehicle(pc, veh_id, geofenceInfo)              
            # publish_end = time.monotonic()
            # elapsed_time = publish_end - publish_start
            # print(f"publish elapsed time: {elapsed_time}")
            # time.sleep(1-elapsed_time)

        # dispatch_end = time.monotonic()
        # dispath_elapse_time = dispatch_end - dispatch_start
        # print(f"dispath_elapse_time: {dispath_elapse_time}")

        event_occur_steps.clear()             

        # print(f"veh_count: {veh_count}")
        # waiting_ack_start = time.monotonic()
        while(True):            
            if Vehicle_dispatcher.ack_count == veh_count:
                Vehicle_dispatcher.ack_count = 0
                break

        # waiting_ack_end = time.monotonic()
        # waiting_ack_elapse_time = waiting_ack_end - waiting_ack_start
        # print(f"waiting_ack_elapse_time: {waiting_ack_elapse_time}")
        
        current_simulation_step += 1
        traci.simulationStep()


    garbage_collector(physical_mqtt_server='127.0.0.1', physical_mqtt_port=7883, virtual_mqtt_server='127.0.0.1', virtual_mqtt_port=7884, traci_simulation=traci.simulation, vehicleDict=vehicleDict)

    print("Simulation Done!")

    traci.close()
