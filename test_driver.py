from PingSonar import Driver
import argparse


parser = argparse.ArgumentParser(description="Input configuration for testing ping driver.")
parser.add_argument('device', type=str, nargs='?', help='device port to use', default='/dev/ttyUSB0')
parser.add_argument('range_min', type=int, nargs='?', help="minimum range (mm)", default=500)
parser.add_argument('range_max', type=int, nargs='?', help="maximum range (mm)", default=2000)
parser.add_argument('ping_count', type=int, nargs='?', help="number of pings", default=100)

args = parser.parse_args()

range_min = args.range_min
range_max = args.range_max
pings = args.ping_count
device = args.device

driver = Driver(device)
driver.start()
protocol_req = driver.pack(6, 5)
driver.send_pkt(protocol_req)
gain_set = driver.pack(1005, 6)
driver.send_pkt(gain_set)
mode_req = driver.pack(1003, 0)
driver.send_pkt(mode_req)
range_req = driver.pack(1001, range_min, range_max)
driver.send_pkt(range_req)
speed_config = driver.pack(1002, 343000)
driver.send_pkt(speed_config)
ping_int_config = driver.pack(1004, 250)
start_profile_pkt = driver.pack(1400, 1300)
driver.send_pkt(start_profile_pkt)
c = 0
profile = []
ranging = []

try:
    while True:
        if driver.profile_available:
            data = driver.get_profile()
            print(data["distance"])
except KeyboardInterrupt:
    stop_profile_pkt = driver.pack(1401, 1300)
    driver.send_pkt(stop_profile_pkt)
    driver.stop()
