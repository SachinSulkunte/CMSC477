# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import robomaster
from robomaster import robot
import time

distance = 0

def sub_data_handler(sub_info):
    distances = sub_info
    distance = distances[0]
    print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distances[0], distances[1], distances[2], distances[3]))
    avoid_obstacle(distance)

#
def avoid_obstacle(distance):
    if distance < 240:
        ep_chassis.move(x=-0.1, y=0.0, z=0).wait_for_completed()
    return


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    #add to init
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)



    time.sleep(60)
    ep_sensor.unsub_distance()
    ep_robot.close()