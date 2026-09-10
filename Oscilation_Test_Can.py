# to be used with CANgaroo Software
import cangaroo
import math
import struct
import time

STEPS = 400

for iface in cangaroo.interfaces():
    print(f"Interface {iface['id']}: {iface['name']}")

while(True):
    for i in range(STEPS):
        phase = (i / STEPS) * (2 * math.pi)
        value_rad = math.sin(phase) * math.pi

        raw_bytes = struct.pack('<f', value_rad)

        msg = cangaroo.Message()
        msg.id = 0x001
        msg.dlc = 4
        msg.set_data(raw_bytes)
        cangaroo.send(msg, interface_id=256)

        print(f"Sent {value_rad:+.3f} rad -> Bytes: {raw_bytes.hex(' ')}")
        time.sleep(0.01)
