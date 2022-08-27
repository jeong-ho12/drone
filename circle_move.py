#!/usr/bin/env python3
import numpy as np
import rospy
import asyncio
import time
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)
from std_msgs.msg import Float32MultiArray


class kkk:
    def __init__(self):
        rospy.init_node('talker',anonymous=True) 
        self.drone = System()
        self.posvel_ned=np.array([0,0,0,0,0,0])

    async def connect2(self):
        await self.drone.connect(system_address="serial:///dev/ttyUSB0:921600")

    async def run(self):
        """ Does Offboard control using velocity body coordinates. """
        await self.connect2()
        

        # print("Waiting for drone to connect...")
        # async for state in self.drone.core.connection_state():
        #     if state.is_connected:
        #         print(f"Drone discovered!")
        #         break

        print("-- Arming")
        await self.drone.action.arm()

        print("-- Setting initial setpoint")
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                  {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return

        print("-- Turn clock-wise and climb")
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, -1.0, 0))
        await asyncio.sleep(8)

        start_time = time.time()
        while True:
            if time.time()-start_time > 5:
                break
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 1.0, 0.0, -11.4591559026))
            await asyncio.sleep(0.1)

    async def telem_posvel(self):
        '''
        Telemetry : position velocity ned
        '''
        await asyncio.sleep(3)
        # print('11111111111111111111111111111111111')
        
        publisher =rospy.Publisher('array',Float32MultiArray,queue_size=1)
        random_list=Float32MultiArray()
        await asyncio.sleep(0.01)
        
        async for pos_ned in self.drone.telemetry.position_velocity_ned():

            self.posvel_ned[0] = pos_ned.position.north_m
            self.posvel_ned[1] = pos_ned.position.east_m
            self.posvel_ned[2] = pos_ned.position.down_m
            self.posvel_ned[3] = pos_ned.velocity.north_m_s
            self.posvel_ned[4] = pos_ned.velocity.east_m_s
            self.posvel_ned[5] = pos_ned.velocity.down_m_s

            random_list.data = [self.posvel_ned[0],self.posvel_ned[1],self.posvel_ned[2]]
            publisher.publish(random_list)
            


if __name__ == "__main__":
    k= kkk()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(asyncio.gather(k.run(), k.telem_posvel()))
