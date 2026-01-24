# Step 1: CAN Sniffer (Pure observation)
import can

bus = can.Bus(channel='can0', bustype='socketcan')
print("Observing CAN traffic...")

for msg in bus:
    print(f"ID: {hex(msg.arbitration_id)} Data: {msg.data.hex()}")
    # NO SENDING - JUST WATCHING