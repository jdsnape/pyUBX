import logging
from queue import Queue
import argparse
import os
import serial
from UBXManager import UBXManager
from threading import Lock
from time import sleep
import UBX
from FSM import FSM_Get, isObj, isACK
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np
import sqlite3
from blessed import Terminal

gnssId_lookup = {
        0: 'GPS',
        1: 'SBAS',
        2: 'Galileo',
        3: 'BeiDou',
        4: 'IMES',
        5: 'QZSS',
        6: 'GLONASS'
        }


spoofDetState = {
        0: 'Unknown',
        1: 'False',
        2: 'True',
        3: 'Multiple'
        }

# use ggplot style for more sophisticated visuals
plt.style.use('ggplot')


class graph:
    def __init__(self, size):
        self.size = size
        self.x_vec = np.linspace(0, 1, self.size+1)[0:-1]
        self.y_vecs = {}
        self.lines = {}
        self.initialized = False

    def update_graph(self, new_vals):
        logging.debug("Updating with new vals: {}".format(new_vals))

        # take new values and append to vectors
        for k, v in new_vals.items():
            if k in self.y_vecs:
                self.y_vecs[k][-1] = v
            else:
                self.y_vecs[k] = np.empty(len(self.x_vec))
                self.y_vecs[k].fill(v)

        self.live_plotter(self.x_vec, self.y_vecs, self.lines)
        for vec in self.y_vecs:
            # This is slightly problematic, as instead of skipping value (or setting to zero) it will put a fake value in)
            # TODO - need a way of putting a null value in (or not plotting this line later?)
            self.y_vecs[vec] = np.append(self.y_vecs[vec][1:], self.y_vecs[vec][-1])

    def live_plotter(self, x_vec, y_data, lines, identifier='Pseudoranges', pause_time=0.1):
        # check we have an axis for each key...
        if self.initialized is not True:
            # this is the call to matplotlib that allows dynamic plotting
            plt.ion()
            self.fig = plt.figure(figsize=(13, 6))
            self.ax = self.fig.add_subplot(111)
            # create a variable for the line so we can later update it
            for vec in y_data:
                line, = self.ax.plot(x_vec, y_data[vec], '-o', alpha=0.8, label=str(vec))
                self.lines[vec] = line
            self.ax.legend()
            plt.ylabel('pseudorange (m)')
            plt.title('Title: {}'.format(identifier))
            plt.show()
            self.initialized = True

        # after the figure, axis, and line are created, we only need to update the y-data
        for vec in self.y_vecs:
            if vec not in lines:
                line, = self.ax.plot(x_vec, y_data[vec], '-', alpha=0.8, label=str(vec))
                self.lines[vec] = line
                self.ax.legend()
            else:
                self.lines[vec].set_ydata(self.y_vecs[vec])

        # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
        plt.pause(pause_time)


@FSM_Get(UBX.MON.VER)
class FSM_VER_Get:
    # Expand this class to be able to handle the response?
    pass


@FSM_Get(UBX.SEC.UNIQID)
class FSM_UNIQID_Get:
    # Expand this class to be able to handle the response?
    pass


class FSM_MSG_Set:
    """FSM handling RAW SET"""
    _class = UBX.CFG.MSG._class
    _id = UBX.CFG.MSG._id

    class STATE(Enum):
        START = 0
        WAIT_GET = 1
        WAIT_GET_ACK = 2
        WAIT_SET_ACK = 3
        DONE = 4

    def __init__(self, msgClass, msgID, rate):
        self.state = FSM_MSG_Set.STATE.START
        self._gottenObj = None
        self._msgClass = msgClass
        self._msgID = msgID
        self._rate = rate

    def done(self):
        return self.state == FSM_MSG_Set.STATE.DONE

    def onUBX(self, obj, manager):
        # Ignore RXM messages (receiver manager messages) as they are periodic (there may be others? - NAV, MON, HNR, ESF)
        # inverse - check if is CFG or ACK message..
        if isObj(obj, UBX.CFG.MSG) or isObj(obj, UBX.ACK.ACK) or isObj(obj, UBX.ACK.NAK):
            if self.state == FSM_MSG_Set.STATE.START:
                if isObj(obj, UBX.CFG.MSG):
                    self._gottenObj = obj
                    self.state = FSM_MSG_Set.STATE.WAIT_GET_ACK
                else:
                    # print("Didnt' get a UBX.CFG.MSG - got a {}".format(obj.__class__))

                    self.state = FSM_MSG_Set.STATE.DONE
            elif self.state == FSM_MSG_Set.STATE.WAIT_GET_ACK:
                if isACK(obj):
                    self._gottenObj.msgClass = self._msgClass
                    self._gottenObj.msgID = self._msgID
                    self._gottenObj.rate_usb = self._rate

                    manager.send(self._gottenObj.serialize())
                    self.state = FSM_MSG_Set.STATE.WAIT_SET_ACK
                else:
                    print("Ooops that went wrong, I was expecting an ACK")
                    self.state = FSM_MSG_Set.STATE.DONE
            elif self.state == FSM_MSG_Set.STATE.WAIT_SET_ACK:
                if isACK(obj):
                    self.state = FSM_MSG_Set.STATE.DONE
                else:
                    print("Ooops that went wrong, I was expecting an ACK")
                    self.state = FSM_MSG_Set.STATE.DONE
            else:
                raise Exception("The FSM went boink.")
        else:
            logging.debug("Ignoring {} message as in config mode".format(obj.__class__))


class Manager(UBXManager):
    def __init__(self, ser, queue,  debug=False):
        UBXManager.__init__(self, ser, debug)
        self._lock = Lock()
        self._fsm = None
        self._dumpNMEA = False
        self._q = queue
        self._dumpRAW = False

    def setDumpRAW(self, val):
        with self._lock:
            self._dumpRAW = val
        logging.debug("dumpRAW: {}".format(self._dumpRAW))

    def setDumpNMEA(self, val):
        with self._lock:
            self._dumpNMEA = val
        if self.debug:
            print("dumpNMEA={}".format(val))

    def onNMEA(self, buffer):
        with self._lock:
            dump = self._dumpNMEA
        if dump:
            print("NMEA: {}".format(buffer))

    def onUBX(self, obj):
        with self._lock:
            if self._fsm is not None:
                self._fsm.onUBX(obj, self)
                if self._fsm.done():
                    logging.debug("Putting {} message on queue".format(str(obj.__class__)))
                    # Put message on queue
                    self._q.put(obj)
                    self._fsm = None
            else:
                # put message on queue
                # print(obj)
                if self._dumpRAW:
                    logging.debug("NON-FSM Putting {} message on queue".format(str(obj.__class__)))
                    # Put message on queue
                    self._q.put(obj)

    def onUBXError(self, msgClass, msgId, errMsg):
        logging.error(msgClass, msgId, errMsg)

    def done(self):
        with self._lock:
            return self._fsm is None

    def waitUntilDone(self, timeout=None):
        sleepTime = 0
        while not self.done():
            sleep(0.5)
            sleepTime += 0.5
            if timeout is not None and sleepTime > timeout:
                return False
        return True

    def VER_GET(self):
        msg = UBX.MON.VER.Get().serialize()
        with self._lock:
            self._fsm = FSM_VER_Get()
        self.send(msg)

    def UNIQID_GET(self):
        msg = UBX.SEC.UNIQID.Get().serialize()
        with self._lock:
            self._fsm = FSM_UNIQID_Get()
        self.send(msg)

    def MSG_EN(self, msgClass, msgID, usbRate):
        msg = UBX.CFG.MSG_GET(bytes([msgClass, msgID]))  # b'\x02\x15')
        with self._lock:
            self._fsm = FSM_MSG_Set(msgClass, msgID, usbRate)
        self.send(msg.serialize())


def num_repeated_sections(item):
    # Get the object module symbol table - we use this to work out what fields are present
    once = item.Fields.__dict__.items()
    # We ignore the 'Repeated' field, and any builtin methods that start with __
    once_fields = list(filter(lambda x: True if not x[0].startswith('__') and not x[0] == 'Repeated' else False, once))
    # Add up all the sizes of the once fields
    once_size = sum(list(map(lambda x: x[1]._size, once_fields)))

    if 'Repeated' in item.Fields.__dict__:
        # Get the repeated field (we only go one level down)
        repeated = item.Fields.__dict__.get('Repeated').__dict__.items()
        # Again - ignore repeated field (second level if exists) and builtin methods
        repeated_fields = list(filter(lambda x: True if not x[0].startswith('__') and not x[0] == 'Repeated' else False, repeated))
        # Add up all teh sizes of the repeated fields
        repeated_size = sum(list(map(lambda x: x[1]._size, repeated_fields)))

        num_repeated_sections = int((item._len - once_size) / repeated_size)
    else:
        num_repeated_sections = 0

    # message len should equal once_size + N*repeated_size
    return num_repeated_sections


def startup_msg(term, msg):
    # Assume screen has been cleared
    print(term.center(msg).rstrip())


def refresh_main_display(term, screen_objects):
    logging.debug("Screen refresh...")
    print(term.clear())
    print(term.ljust(' '))
    for k, v in screen_objects.items():
        print(term.move_x(v['pos'][0]) + term.move_y(v['pos'][1]) + term.bold(k + ': ') + v['value'])


if __name__ == '__main__':
    logging.basicConfig(filename='UBXConsumer.log', level=logging.DEBUG)
    parser = argparse.ArgumentParser(
            description='Consume raw measumement data from a u-blox M8 device'
            )

    parser.add_argument(
                        '-v', '--verbose', dest='debug', action='store_true',
                        help='Turn on debug mode.'
                        )

    parser.add_argument('device', type=str,
                        help='receiver device name'
                        )

    parser.add_argument(
                '-d', '--database', dest='database', action='store_true',
                help='output data to database (results.db)'
            )

    parser.add_argument(
            '-g', '--graph', dest='graph', action='store_true',
            help='show a graph of measurements'
            )
    args = parser.parse_args()

    debug = (os.environ.get("DEBUG") is not None) or args.debug

    if args.database:
        logging.debug("Opening database")
        s = sqlite3.connect('results.db')
        cursor = s.cursor()

    # if debug:
    logging.basicConfig(filename='UBXConsumer.log', level=logging.DEBUG)
    # else:
    #    logging.basicConfig(filename='UBXConsumer.log', level=logging.DEBUG)  # TODO - can't use --database and ==verbose together??

    try:
        term = Terminal()
        with term.fullscreen():
            # Move cursor to the middle
            print(term.move_y((term.height // 2) - 1))
            # Create a dict to hold information about what's on the screen - TODO update on each 'tick'
            screen_objects = {
                    'Outputs': {'value': '', 'pos': (1, term.height)},
                    'Module': {'value': '', 'pos': (term.width // 2, term.height)},
                    'Position': {'value': '', 'pos': (1, 1)},
                    'Time': {'value': '', 'pos': (term.width//2 - 20, 1)},
                    'Spoofing': {'value': '?', 'pos': (term.width - 20, 1)},
                    'Satellite Info': {'value': '\n\ta', 'pos': (1, 3), 'data': {'GPS': {}, 'GLONASS': {}, 'Galileo': {}}},
                    'DeviceID': {'value': '', 'pos': (term.width - 25, term.height)}

                    }

            if args.database:
                screen_objects['Outputs']['value'] = 'Database'
            else:
                screen_objects['Outputs']['value'] = 'None'

            fields_to_save = {
                    UBX.RXM.RAWX: {
                        'once': ['rcvrTow', 'leapS'],
                        'repeated': ['prMeas', 'cpMeas', 'doMes', 'gnssId', 'svId', 'cno', 'prStdev', 'cpStdev', 'doStdev', 'trkStat'],
                        'TOW': 'rcvrTow',
                        'TOW_scale': 1000
                        },
                    UBX.RXM.MEASX: {
                        'once': ['gpsTOW'],
                        'repeated': ['cNo', 'mpathIndic', 'dopplerMS', 'dopplerHz', 'gnssId', 'svId', 'wholeChips', 'fracChips', 'codePhase', 'intCodePhase', 'pseuRangeRMSErr'],
                        'TOW': 'gpsTOW',
                        'TOW_scale': 1
                        },
                    UBX.NAV.SAT: {
                        'once': ['iTOW'],
                        'repeated': ['gnssId', 'svId', 'cno', 'elev', 'azim', 'prRes'],
                        'TOW': 'iTOW',
                        'TOW_scale': 1
                        },
                    UBX.NAV.STATUS: {
                        'once': ['iTOW', 'flags2'],
                        'repeated': [],
                        'TOW': 'iTOW',
                        'TOW_scale': 1
                        },
                    UBX.NAV.PVT: {
                        'once': ['iTOW', 'year', 'month', 'day', 'hour', 'minute', 'sec', 'lat', 'lon'],
                        'repeated': [],
                        'TOW': 'iTOW',
                        'TOW_scale': 1
                        }
                    }

            startup_msg(term, "Attaching to {}".format(args.device))
            # logging.debug("Attaching to {}".format(args.device))
            ser = serial.Serial(args.device, 9600, timeout=None)
            queue = Queue(maxsize=10)
            manager = Manager(ser, queue, debug=debug)
            manager.setDumpNMEA(False)

            startup_msg(term, "Starting Manager")
            # logging.debug("Starting manager...")
            manager.start()
            # Wait for manager to init
            sleep(1)

            # Get a unique ID to identify the device
            manager.UNIQID_GET()

            item = queue.get()
            if type(item).__name__ != 'UNIQID':
                logging.error("Got a {} msg when expecting a VER message".format(type(item).__bases__))
                raise Exception  # TODO exception type

            # Assembly the uniqueID
            uniqueId = "{:x}:{:x}:{:x}:{:x}:{:x}".format(item.uniqueId_1, item.uniqueId_2, item.uniqueId_3, item.uniqueId_4, item.uniqueId_5)

            startup_msg(term, "Device Unique ID: {}".format(uniqueId))
            screen_objects['DeviceID']['value'] = uniqueId

            # Check that we have a version we can handle (good test of whether device is up and running...)
            logging.debug("Getting version...")
            manager.VER_GET()

            # wait until item appears on queue - TODO shldn't block forever
            item = queue.get()
            if type(item).__name__ != 'VER':
                logging.error("Got a {} msg when expecting a VER message".format(type(item).__bases__))
                raise Exception  # TODO exception type
            # Check we have a supported device
            swVersion = float(item.swVersion.split(' ')[2])
            if item.extension_4 != 'MOD=NEO-M8T-0' or swVersion < 3.01:
                logging.error("Wrong module")
                raise Exception

            screen_objects['Module']['value'] = item.extension_4 + ', ' + str(swVersion)
            startup_msg(term, "Got correct module ({} - {}), enabling raw messages".format(item.extension_4, swVersion))

            # logging.info("Got correct version info - enabling raw messages ({} - {})".format(item.extension_4, swVersion))
            # TODO - Should we do a cold start here?
            # TODO - stop NMEA messages?
            # TODO - change baud rate??
            # Enable the messages we want to consume

            for msg_type in fields_to_save.keys():
                manager.MSG_EN(msg_type._class, msg_type._id, 0x01)
                item = queue.get()  # Block till get response on queue
                if type(item).__name__ != 'ACK':
                    logging.error("Unable to enable {} messages".format(msg_type.__name__))
                    raise Exception

                startup_msg(term, "{} messages enabled".format(msg_type.__name__))
                logging.info("Got ACK for setting {} messages".format(msg_type.__name__))

            manager.setDumpRAW(True)

            if args.graph:
                g = graph(3500)

            if args.database:
                startup_msg(term, "Initialising database")
                # table creation string
                for msg_type in fields_to_save:
                    name = msg_type.__name__
                    # TODO - types
                    query_substring = '('
                    for field in fields_to_save[msg_type]['once']:
                        if field != fields_to_save[msg_type]['TOW']:  # ignore any TOW field
                            query_substring += field+' REAL,'
                    for field in fields_to_save[msg_type]['repeated']:
                        query_substring += field+' REAL,'

                    query_substring += 'TOW INTEGER, DEVICE_ID TEXT);'
                    query = 'CREATE TABLE IF NOT EXISTS '+name+'_results '+query_substring
                    startup_msg(term, "Creating {} table".format(name))
                    cursor.execute(query)
                    s.commit()

            objects_to_process = list(fields_to_save.keys())

            combined_results = {}
            old_tow = 0
            while True:
                results = {}

                item = queue.get()

                # Get fields from message (do we not have a public to do this already?)
                logging.debug("Got {} message from queue".format(item.__class__))
                # Check it's a message type we want to process (objects_to_process derived from our dict of fields to save)
                if type(item) in objects_to_process:
                    msg_type = item.__class__.__name__

                    # extract fields of interest
                    for field in fields_to_save[type(item)]['once']:
                        results[field] = getattr(item, field)

                    results['repeated'] = []

                    for i in range(1, num_repeated_sections(item)+1):
                        tmp = {}

                        for field in fields_to_save[type(item)]['repeated']:
                            if field.startswith('gnssId'):
                                tmp[field] = gnssId_lookup[getattr(item, field+'_'+str(i))]
                            else:
                                tmp[field] = getattr(item, field+'_'+str(i))
                        results['repeated'].append(tmp)

                    # work out the TOW (normalise the fields)
                    new_tow = int(results[fields_to_save[type(item)]['TOW']]) * fields_to_save[type(item)]['TOW_scale']

                    # Update screen objects
                    if msg_type == 'STATUS':
                        screen_objects['Spoofing']['value'] = spoofDetState[(results['flags2'] & 24) >> 3]

                    if msg_type == 'PVT':
                        lat = results['lat'] * 1e-7
                        lon = results['lon'] * 1e-7
                        screen_objects['Position']['value'] = str(round(lat, 4)) + ' ' + str(round(lon, 4))
                        screen_objects['Time']['value'] = str(results['year']) + '-' + str(results['month']) + \
                            '-' + str(results['day']) + ' ' + str(results['hour']) + ':' + \
                            str(results['minute']) + ':' + str(results['sec'])

                    if msg_type == 'RAWX':
                        # Update satellite info - this will get parsed into the right format for display when we have a TOW rollover
                        for key in screen_objects['Satellite Info']['data']:
                            # Need to remove readings from data dict which are no longer in results - do this by clearing all the data out.
                            # TODO - this won't work if want to include info from other messages
                            screen_objects['Satellite Info']['data'][key] = {}
                        for res in results['repeated']:
                            screen_objects['Satellite Info']['data'][res['gnssId']][res['svId']] = {'cpMeas': res['cpMeas'], 'prMeas': res['prMeas']}

                            if res['cpStdev'] == 15.0:
                                screen_objects['Satellite Info']['data'][res['gnssId']][res['svId']]['cpStdev'] = 'Invalid'
                            else:
                                screen_objects['Satellite Info']['data'][res['gnssId']][res['svId']]['cpStdev'] = res['cpStdev']

                    # how to represent in DB? one table per type,
                    if args.database:
                        #  TODO - there must be a better way of doing this :)
                        # Save current results set
                        query_front = 'INSERT INTO '+msg_type+'_results'
                        col_string = ' ('
                        vals = []
                        val_string = ' VALUES ('
                        # extract the 'once' fields
                        for key, val in results.items():
                            if key != 'repeated' and key != fields_to_save[item.__class__]['TOW']:
                                # Need to ignore e.g. iTOW field.
                                col_string += key+','
                                vals.append(val)
                                val_string += '?,'

                        if len(results['repeated']) > 0:
                            # Need to do one insert per repeated entry
                            for repeated_item in results['repeated']:
                                tmp_col_string = col_string
                                tmp_vals = []
                                tmp_vals += vals
                                tmp_val_string = val_string

                                for key, val in repeated_item.items():
                                    tmp_col_string += key+','
                                    tmp_vals.append(val)
                                    tmp_val_string += '?,'

                                tmp_col_string += 'TOW, DEVICE_ID)'
                                tmp_vals.append(new_tow)
                                tmp_vals.append(uniqueId)
                                tmp_val_string += '?,?)'
                                query_string = query_front + tmp_col_string + tmp_val_string+';'
                                cursor.execute(query_string, tuple(tmp_vals))

                        else:
                            col_string += 'TOW, DEVICE_ID)'
                            vals.append(new_tow)
                            vals.append(uniqueId)
                            val_string += '?,?)'
                            query_string = query_front + col_string + val_string+';'
                            logging.debug("QUERY: {}".format(query_string))
                            logging.debug("vals: {}".format(vals))
                            cursor.execute(query_string, tuple(vals))

                        s.commit()

                    if new_tow > old_tow:
                        # output existing data
                        logging.debug("Have rollover - new_tow: {}, old_tow: {}".format(new_tow, old_tow))

                        # This is where to update the screen..First we update the satellite info we display - this is a bit overcomplicated
                        screen_objects['Satellite Info']['value'] = term.bold + \
                            '\n{:>14} \t {:18} \t\t {:18} \t\t {}\n'.format('svID', 'Pseudorange', 'Carrier Phase', 'CP stdev') + term.normal

                        for key in screen_objects['Satellite Info']['data']:
                            for i in range(1, 33):  # There is svId=255 for GLONASS but ignoringn
                                if i in screen_objects['Satellite Info']['data'][key]:
                                    screen_objects['Satellite Info']['value'] += "{:>10} {:>2}: \t {} \t\t {} \t\t {}\n"\
                                                                                .format(
                                                                                        key, str(i), screen_objects['Satellite Info']['data'][key][i]['prMeas'],
                                                                                        screen_objects['Satellite Info']['data'][key][i]['cpMeas'],
                                                                                        screen_objects['Satellite Info']['data'][key][i]['cpStdev']
                                                                                        )
                                else:
                                    screen_objects['Satellite Info']['value'] += "{:>10} {:>2}: \t {:18} \t\t {:18} \t\t {}\n".format(key, str(i), '-', '-', '-')

                        refresh_main_display(term, screen_objects)
                        old_tow = new_tow

    finally:
        logging.info("Shutting down...")
        manager.shutdown()

        if ser is not None:
            ser.close()
        logging.info("Bye")
