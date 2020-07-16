import serial
import threading
import struct
import json


class Driver:
    """
    Class that interfaces with Blue Robotics sonar via serial object inheritance. Borrows message definitions from
    the ping-protocol repository (https://github.com/bluerobotics/ping-protocol.git).
    """

    def __init__(self, dev, baud=115200, timeout=0.5):
        with open("./ping-protocol/src/definitions/common.json", 'r') as json_file:
            self.common_templates = json.load(json_file)
        with open("./ping-protocol/src/definitions/ping1d.json", 'r') as json_file:
            self.ping1d_templates = json.load(json_file)
        self._compile_msgs()
        # Initialise the serial object
        self.ser = serial.Serial(port=dev,
                                 baudrate=baud,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=timeout,
                                 xonxoff=False,
                                 rtscts=False,
                                 write_timeout=None,
                                 dsrdtr=False,
                                 inter_byte_timeout=None,
                                 exclusive=None)
        self._preamble = struct.Struct("<2B2H2B")
        self._checksum = struct.Struct("<H")
        self._listener = threading.Thread(target=self._receive_pkt, name="Thread-SerialListener")
        self._run = False
        self.profile = {"distance": -1,
                        "confidence": -1,
                        "transmit_duration": -1,
                        "ping_number": -1,
                        "scan_start": -1,
                        "scan_length": -1,
                        "gain_setting": -1,
                        "profile_data_length": -1,
                        "profile_data": -1}
        self.profile_available = False

    def get_profile(self):
        self.profile_available = False
        return self.profile

    def start(self):
        self._run = True
        if self.ser.closed:
            self.ser.open()
        self._listener.start()

    def stop(self):
        self._run = False

    def _compile_msgs(self):
        # TODO Handle deprecated messages (perhaps flag warning or raise exception).
        self.msg_templates = dict()
        # Compile the common messages
        msg_types = self.common_templates["messages"]
        for msg_type in msg_types:
            msgs = msg_types[msg_type]
            for msg in msgs.keys():
                dynamic = False
                payload_msg = ""
                for item in msgs[msg]["payload"]:
                    payload_msg = payload_msg + self._parse_item(item)
                if "@" in payload_msg:
                    dynamic = True
                else:
                    payload_msg = struct.Struct(payload_msg)
                self.msg_templates[msgs[msg]["id"]] = {"payload": payload_msg,
                                                       "name": msg,
                                                       "description": msgs[msg]["description"],
                                                       "dynamic": dynamic}
        # Compile the ping1d messages
        msg_types = self.ping1d_templates["messages"]
        for msg_type in msg_types:
            msgs = msg_types[msg_type]
            for msg in msgs.keys():
                dynamic = False
                payload_msg = "<"
                for item in msgs[msg]["payload"]:
                    payload_msg = payload_msg + self._parse_item(item)
                if "@" in payload_msg:
                    dynamic = True
                else:
                    payload_msg = struct.Struct(payload_msg)
                self.msg_templates[msgs[msg]["id"]] = {"payload": payload_msg,
                                                       "name": msg,
                                                       "description": msgs[msg]["description"],
                                                       "dynamic": dynamic}

    def _parse_int_array(self, length):
        return "{}B".format(length)

    def _parse_item(self, item):
        dtypes = {u"u8": "B",
                  u"u16": "H",
                  "char": "s",
                  u"u32": "L"
                  }
        if item["type"] == u'vector':
            return "@"
        elif isinstance(item["type"], unicode):
            return dtypes[item["type"]]
        raise TypeError

    def _receive_pkt(self):
        # listen for start characters
        buff = ""
        while not self.ser.closed:
            # if something detected on the serial port
            if self.ser.in_waiting:
                # read in a byte
                buff = buff + self.ser.read()
                # if the buffer starts with "BR" we have packet start
                if buff.startswith("BR"):
                    buff = buff + self.ser.read(6)
                # if the buffer is longer than or equal to 2 and doesn't start with "BR", reset buffer and keep reading.
                elif len(buff) >= 2:
                    buff = ""
                    continue
                # if the buffer length is not equal or greater than 2 yet, keep reading
                else:
                    continue
                # the preamble should be read in at this point, so unpack it
                try:
                    preamble = self._preamble.unpack(buff)
                except:
                    print buff
                    buff = ""
                    continue
                length = preamble[2]
                msg_id = preamble[3]
                src_id = preamble[4]
                dst_id = preamble[5]
                # read in the payload
                payload = self.ser.read(length)
                # read in the checksum
                checksum = self.ser.read(2)
                checksum = self._checksum.unpack(checksum)[0]
                template = self.msg_templates[unicode(msg_id)].copy()
                unpacked = self.unpack(template, buff, payload, length, checksum)
                buff = ""
                if msg_id == 1300:
                    self.profile.update(distance=unpacked[0],
                                        confidence=unpacked[1],
                                        transmit_duration=unpacked[2],
                                        ping_number=unpacked[3],
                                        scan_start=unpacked[4],
                                        scan_length=unpacked[5],
                                        gain_setting=unpacked[6],
                                        profile_data_length=unpacked[7],
                                        profile_data=unpacked[8:])
                    self.profile_available = True
        self.ser.close()

    def unpack(self, template, preamble, payload, length, checksum):
        if template['dynamic']:
            tmp = template['payload'].replace('@', '')
            template['payload'] = struct.Struct(template['payload'].replace('@', self._parse_int_array(length-struct.Struct(tmp).size)))
        if sum(map(ord, preamble+payload)) != checksum:
            print "CHECKSUM ERROR"
        return template['payload'].unpack(payload)

    def pack(self, msg_id, *contents):
        payload = self.msg_templates[unicode(msg_id)]
        pkt = payload["payload"].pack(*contents)
        length = payload["payload"].size
        preamble = self._preamble.pack(ord('B'), ord('R'), length, msg_id, 0, 0)
        checksum = self._checksum.pack(sum(map(ord, preamble + pkt)))
        return preamble + pkt + checksum

    def send_pkt(self, pkt):
        self.ser.write(pkt)
