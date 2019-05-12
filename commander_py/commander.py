#!/usr/bin/env python

# ----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
# ----------------------------------------------------------------------------------------------------------------------#
#   Christian Karlsson
#   Kandidatarbete COBOTS
#   V.2.0.0.
# ----------------------------------------------------------------------------------------------------------------------#

import rclpy
import time

from commander_msgs.msg import Command
from commander_msgs.msg import State

# Used for parsing additional arguments to callback function
from functools import partial

# Generating product order for testing purposes
import commander_py.generateProductOrder as gpo

# Defining publishers and subscribers
publishers = []
subscribers = []

# Defining products, a station array and work order array
products = ["Sedan", "Jeep"]
NUMBEROFPRODUCTS = 2

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

# Number of stations, hardcoded/predefined
NO_OF_STATIONS = 3

# Sleep time where sleep is implemented
SLEEP_TIME = 1

# Messages for the 3 different stations and AGV
COMMAND_ARRAY = ["Kit", "Assemble", "Screw", "Move AGV to station"]

# Tracks which station the agv is on
agv_pos = 0


def main(args=None):
    input("Press Enter to continue...")

    # Initializes node, creates publishers, subscribers and initializes the
    # message arrays
    global node
    node = initialize(NO_OF_STATIONS)

    # Generate product order
    product_order = gpo.generate_product_order(
        NUMBEROFPRODUCTS, products)

    # Puts products in work_order in a hardcoded/predefined pattern
    create_work_order(product_order)

    # While there is something in at least one of the stations or if the work
    # order is not empty
    all_stations_empty = is_all_stations_empty()

    while not all_stations_empty or len(work_order) != 0:

        # From last station index down to 0
        for station in range(NO_OF_STATIONS-1, -1, -1):

            print("Checking station {}".format(str(station)))
            rclpy.spin_once(node)
            time.sleep(SLEEP_TIME)

            # If station is not empty
            if cmd_msgs[station].product_name:

                print("Checking if station {} is done".format(str(station)))
                time.sleep(SLEEP_TIME)

                if check_state(station) == FINISHED and agv_pos == station:
                    # Last station => no station to send forward to
                    if station == NO_OF_STATIONS - 1:
                        station_done(station)

                    # Not last station => check forward
                    elif check_state(station+1) == INIT:
                        cmd_msgs[station+1].product_name = (cmd_msgs[station].
                                                            product_name)

                        # Should a station be done before the next station has
                        # started on the product?
                        station_done(station)
                        send_command(station+1)

                    else:
                        print("Station {} is done but the \
                              station ahead is not ready".format(str(station)))

                else:
                    print('''Station {} is not done, or the AGV is not in
                            position'''.format(str(station)))

            else:
                # If nothing in the first station but the work_order is not
                # empty
                if station == 0 and len(work_order) != 0:
                    if check_state(station) != INIT:

                        print("Station {} not ready for new work order".format(
                            str(station)))

                    else:
                        print("Send in the next order to station {}".format(
                              str(station)))
                        cmd_msgs[station].product_name = work_order.pop(0)
                        send_command(station)
                else:
                    print("Nothing to do")
                    time.sleep(SLEEP_TIME)

        all_stations_empty = is_all_stations_empty()

    print(work_order)

    node.destroy_node()
    rclpy.shutdown()


def state_callback(station, state_msg: str):
    state_msgs[station] = state_msg


# Initializes node, creates publishers and subscribers and initializes the
# message arrays
def initialize(NO_OF_STATIONS: int):
    rclpy.init()
    node = rclpy.create_node('command_node')

    for i in range(NO_OF_STATIONS):
        publishers.append(node.create_publisher(
            Command, 'cmd{}'.format(str(i))))

        cmd_msgs.append(Command())
        state_msgs.append(State())

        subscribers.append(node.create_subscription(State, 'state{}'.format(
            str(i)), partial(state_callback, i)))

    # Below is only for the AGV
    publishers.append(node.create_publisher(Command, 'cmdA'))
    cmd_msgs.append(Command())
    state_msgs.append(State())
    subscribers.append(node.create_subscription(State, 'stateA', partial(
        state_callback, NO_OF_STATIONS)))

    return node


# Puts products in work_order in a hardcoded/predefined pattern
def create_work_order(Product_order: list):
    for product in Product_order:
        work_order.append(product)


# Returns True if all stations are empty, False otherwise
def is_all_stations_empty():
    rclpy.spin_once(node)
    for cmd_msg in cmd_msgs:
        if cmd_msg.product_name != '':
            return False

    return True


# Checks and returns the state of the station. Throws exception if state is
# not recognized
def check_state(station: int):
    state = state_msgs[station].state.lower()
    if not state_status(state):
        print("Unknown state on station {}: {}".format(str(station), state))

    return state


def state_status(state):
    ''' Check station states against the preapproved states:
        (init, executing, finished) \n
    State: Station state \n
    Returns: (bool) True or false depending on if the input state is found
             in approvedStatus
    '''
    acceptedStatus = [INIT, EXECUTING, FINISHED, '']  # '' to enable empty msgs
    if state in acceptedStatus:
        return True
    else:
        return False


# Sets a station to init with handshake
def station_done(station: int):
    cmd_msgs[station].command = ''
    cmd_msgs[station].run = False
    cmd_msgs[station].product_name = ''

    while check_state(station) != INIT:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        print("Pubing run=false until state=init on station {}".format(
                str(station)))
        time.sleep(SLEEP_TIME)

    move_agv(station)


# Moves the AGV to the next station
def move_agv(station: int):
    if station >= NO_OF_STATIONS - 1:
        next_station = 0
    else:
        next_station = station + 1

    while check_state(NO_OF_STATIONS) != INIT:
        print("Waiting for AGV to be in init state")
        time.sleep(SLEEP_TIME)
        rclpy.spin_once(node)

    print("Moving AGV to station {}".format(next_station))
    cmd_msgs[NO_OF_STATIONS].command = "{}: {}".format(COMMAND_ARRAY[
        NO_OF_STATIONS], str(next_station))
    send_command(NO_OF_STATIONS)

    while check_state(NO_OF_STATIONS) == EXECUTING:
        print("Waiting for AGV to finish")
        time.sleep(SLEEP_TIME)
        rclpy.spin_once(node)

    cmd_msgs[NO_OF_STATIONS].command = ''
    cmd_msgs[NO_OF_STATIONS].run = False
    cmd_msgs[NO_OF_STATIONS].product_name = ''

    while check_state(NO_OF_STATIONS) != INIT:
        publishers[NO_OF_STATIONS].publish(cmd_msgs[NO_OF_STATIONS])
        rclpy.spin_once(node)
        print("Pubing run=false until state=init on AGV")
        time.sleep(SLEEP_TIME)

    global agv_pos
    agv_pos = next_station


# Sends command to station and puts run to true when handshake is established
def send_command(station: int):
    # cmd_msgs[station].command = "Assemble in station {}".format(str(station))
    if station != NO_OF_STATIONS:
        cmd_msgs[station].command = COMMAND_ARRAY[station]

    cmd_msgs[station].run = False

    # Waiting for handshake for the command
    while state_msgs[station].cmd != cmd_msgs[station].command:
        publishers[station].publish(cmd_msgs[station])
        print("Waiting on station {} with the message {}".format(
            str(station), cmd_msgs[station].command))
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME)

    cmd_msgs[station].run = True

    while check_state(station) != EXECUTING:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        print("Pubing run=true until state=executing on station {}".format(
              str(station)))
        time.sleep(SLEEP_TIME)


if __name__ == '__main__':
    main()
