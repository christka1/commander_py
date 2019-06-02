#!/usr/bin/env python

# ----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
# ----------------------------------------------------------------------------------------------------------------------#
#   Christian Karlsson
#   Kandidatarbete COBOTS
#   V.4.0.0.
# ----------------------------------------------------------------------------------------------------------------------#

import rclpy
import time
import os
import configparser

from commander_msgs.msg import Command
from commander_msgs.msg import State

import commander_py.generatelog as log  # for ubuntu
# import generatelog as log               # for everything else

# Used for parsing additional arguments to callback function
from functools import partial

# Read from config file:
CONFIGFILEPATH = "{}/{}".format(os.path.dirname(__file__), 'config.ini')
Config = configparser.ConfigParser()
Config.read(CONFIGFILEPATH)

# Defining publishers and subscribers
publishers = []
subscribers = []

# Input file for productorder:
product_order_input = "/product_order_input.txt"

# First-in-first-out order, gets popped when going to the first station
work_order = []

# Defining message arrays
cmd_msgs = []
state_msgs = []

# Defining the different states allowed
INIT = 'init'
EXECUTING = 'executing'
FINISHED = 'finished'

# Defining the node
node = None

# Number of stations
NO_OF_STATIONS = Config.getint('Station', 'numberOfStations')

# Number of transports
NO_OF_TRANSPORTS = Config.getint('Transport', 'numberOfTransporters')

# Sleep time where sleep is implemented
SLEEP_TIME = Config.getfloat('General', 'SLEEP_TIME')

# Messages for the different stations and transport
COMMAND_ARRAY = Config.get('Station', 'Stationtypes').split(',')
COMMAND_ARRAY = list(map(str.strip, COMMAND_ARRAY))

COMMAND_TRANSPORT = str.strip(Config.get(
    'Transport', 'TransportStationMessage'))

# Tracks which stations the transports are on
transports_pos = []


def main(args=None):
    input("Press Enter to continue...")

    # Initializes node, creates publishers, subscribers and initializes the
    # message arrays
    global node
    node = initialize(NO_OF_STATIONS, NO_OF_TRANSPORTS)

    # Read product order
    path = os.path.dirname(
        os.path.realpath(__file__)) + product_order_input
    file = open(path, "r")
    # Using read() and splitlines() to avoid \n character
    work_order = file.read().splitlines()
    file.close

    # While there is something in at least one of the stations or if the work
    # order is not empty
    all_stations_empty = is_all_stations_empty()

    while not all_stations_empty or len(work_order) != 0:

        # From last station index down to 0
        for station in range(NO_OF_STATIONS-1, -1, -1):

            msg = "Checking station {}".format(str(station))
            log.write_to_log(msg)
            print(msg)
            rclpy.spin_once(node)
            time.sleep(SLEEP_TIME)

            # If station is not empty
            if cmd_msgs[station].product_name:

                msg = "Checking if station {} is done".format(str(station))
                log.write_to_log(msg)
                print(msg)
                time.sleep(SLEEP_TIME)

                if (check_state(station) == FINISHED and
                        is_transport_at_station(station)):
                    # Last station => no station to send forward to
                    if station == NO_OF_STATIONS - 1:
                        station_done(station)

                    # Not last station => check forward
                    elif check_state(station+1) == INIT:
                        cmd_msgs[station+1].product_name = (cmd_msgs[station].
                                                            product_name)
                        station_done(station)
                        send_command(station+1)

                    else:
                        msg = ("Station {} is done but the "
                               "station ahead is not ready").format(str(
                                   station))
                        log.write_to_log(msg)
                        print(msg)

                else:
                    msg = ("Station {} is not done, or the transport is not in"
                           " position").format(str(station))
                    log.write_to_log(msg)
                    print(msg)

            else:
                # If nothing in the first station but the work_order is not
                # empty
                if station == 0 and len(work_order) != 0:
                    if check_state(station) != INIT:

                        msg = "Station {} not ready for new work order".format(
                            str(station))
                        log.write_to_log(msg)
                        print(msg)

                    else:
                        msg = "Send in the next order to station {}".format(
                              str(station))
                        log.write_to_log(msg)
                        print(msg)
                        cmd_msgs[station].product_name = work_order.pop(0)
                        send_command(station)
                else:
                    msg = "Nothing to do"
                    log.write_to_log(msg)
                    print(msg)
                    time.sleep(SLEEP_TIME)

        all_stations_empty = is_all_stations_empty()

    msg = "All stations and work order empty, program finished"
    log.write_to_log(msg)
    print(msg)

    node.destroy_node()
    rclpy.shutdown()


def state_callback(station: int, state_msg):
    '''
    Callback method for all stations \n
    Input:
        station (int) - Which station the message comes from
        state_msg (string) - The message received
    '''
    state_msgs[station] = state_msg


def initialize(NO_OF_STATIONS: int, NO_OF_TRANSPORTS: int):
    '''
    Initializes node, creates publishers and subscribers and initializes the
    message arrays. \n
    Input:
        NO_OF_STATIONS (int) - Number of stations in the production system.
        NO_OF_TRANSPORTS (int) - Number of transport in the production system.
    Returns:
        node - Command_node
    '''
    rclpy.init()
    node = rclpy.create_node('command_node')

    # Generating topics and messages for the stations
    for i in range(NO_OF_STATIONS):
        publishers.append(node.create_publisher(
            Command, 'cmd{}'.format(str(i))))

        cmd_msgs.append(Command())
        state_msgs.append(State())

        subscribers.append(node.create_subscription(State, 'state{}'.format(
            str(i)), partial(state_callback, i)))

        if i == 0:
            transports_pos.append([])
        else:
            transports_pos.append(None)

    # Generating topics and messages for the transports
    for i in range(NO_OF_TRANSPORTS):
        publishers.append(node.create_publisher(
            Command, 'agv_cmd{}'.format(str(i))))

        cmd_msgs.append(Command())
        state_msgs.append(State())

        subscribers.append(node.create_subscription(
            State, 'agv_state{}'.format(str(i)), partial(
                state_callback, i + NO_OF_STATIONS)))

        transports_pos[0].append(i)

    return node


def is_all_stations_empty():
    '''
    Check if all stations are empty. \n
    Returns True or False
    '''
    rclpy.spin_once(node)
    for cmd_msg in cmd_msgs:
        if cmd_msg.product_name != '':
            return False

    return True


def check_state(station: int):
    '''
    Checks and returns the state of the station. Prints message if state is
    not recognized \n
    Input:
        station (int) - Which station to check the state for.
    Returns:
        state - State message of the given station
    '''
    state = state_msgs[station].state.lower()
    if not state_status(state):
        msg = "Unknown state on station {}: {}".format(str(station), state)
        log.write_to_log(msg)
        print(msg)

    return state


def state_status(state):
    ''' Check station states against the preapproved states:
        (init, executing, finished) \n
    State: Station state \n
    Returns: True or False depending on if the input state is found
             in approvedStatus
    '''
    acceptedStatus = [INIT, EXECUTING, FINISHED, '']  # '' to enable empty msgs
    if state in acceptedStatus:
        return True
    else:
        return False


def is_transport_at_station(station: int):
    '''
    Check if there is a transport at the station. \n
    Input:
        station (int) - which station to check for transport
    Returns: True or False
    '''
    if station == 0 and len(transports_pos[0]) > 0:
        return True
    elif station != 0 and transports_pos[station] is not None:
        return True
    return False


def station_done(station: int):
    '''
    Sets a station to init with handshake \n
    Input:
        station (int) - which station is done
    '''
    cmd_msgs[station].command = ''
    cmd_msgs[station].run = False
    cmd_msgs[station].product_name = ''

    msg = "Pubing run=false until state=init on station {}".format(
        str(station))
    log.write_to_log(msg)
    print(msg)
    while check_state(station) != INIT:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)

    move_transport(station)


def move_transport(station: int):
    ''' Moves the correct transport to the next station \n
    Input:
        station (int) - Current station
    '''

    current_agv = None

    if station == 0:
        current_agv = transports_pos[0].pop(0)
    else:
        current_agv = transports_pos[station]
        transports_pos[station] = None

    if station >= NO_OF_STATIONS - 1:
        next_station = 0
        transports_pos[0].append(current_agv)
    else:
        next_station = station + 1
        transports_pos[next_station] = current_agv

    # Making correct offset in array
    index_agv = current_agv + NO_OF_STATIONS

    msg = "Waiting for transport {} to be in init state".format(current_agv)
    log.write_to_log(msg)
    print(msg)
    while check_state(index_agv) != INIT:
        time.sleep(SLEEP_TIME/2)
        rclpy.spin_once(node)

    msg = "Moving transport {} to station {}".format(current_agv, next_station)
    log.write_to_log(msg)
    print(msg)
    cmd_msgs[index_agv].command = "{}: {}".format(
        COMMAND_TRANSPORT, str(next_station))
    send_command(index_agv)

    msg = "Waiting for transport {} to finish".format(current_agv)
    log.write_to_log(msg)
    print(msg)
    while check_state(index_agv) == EXECUTING:
        time.sleep(SLEEP_TIME/2)
        rclpy.spin_once(node)

    cmd_msgs[index_agv].command = ''
    cmd_msgs[index_agv].run = False
    cmd_msgs[index_agv].product_name = ''

    msg = "Pubing run=false until state=init on transport {}".format(
        current_agv)
    log.write_to_log(msg)
    print(msg)
    while check_state(index_agv) != INIT:
        publishers[index_agv].publish(cmd_msgs[index_agv])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)


def send_command(station: int):
    '''Sends command to station and puts run to true when handshake is
    established \n
    Input:
        station (int) - Current station
    '''
    if station < NO_OF_STATIONS:
        cmd_msgs[station].command = COMMAND_ARRAY[station]

    cmd_msgs[station].run = False

    # Waiting for handshake
    msg = "Waiting on station {} with the message {}".format(
        str(station), cmd_msgs[station].command)
    log.write_to_log(msg)
    print(msg)
    while state_msgs[station].cmd != cmd_msgs[station].command:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)

    cmd_msgs[station].run = True

    msg = "Pubing run=true until state=executing on station {}".format(
              str(station))
    log.write_to_log(msg)
    print(msg)
    while check_state(station) != EXECUTING:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)


if __name__ == '__main__':
    main()
