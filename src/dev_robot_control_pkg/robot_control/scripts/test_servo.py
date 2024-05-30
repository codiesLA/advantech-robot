from RS485 import rs_485
import time
def conv_msg(a: int, v: int):
    val = a+65535*(a - v > 1000)*-1*(a!=0)
    # if val < 0: 
    #     val = val*(-1)
    return val
velocity_l = 0
serial_ = rs_485("/dev/ttyUSB0", 1, 8, "E", 115200, 0.5)
client = serial_.connect_()
print(client.connect())
time.sleep(0.1)
registerSpeedHZ = 0xD0
vl_feedback = client.read_holding_registers(address=registerSpeedHZ, count = 2, slave =  0x03)
if vl_feedback.isError():
    print(f"Modbus Error: {vl_feedback}")
else:
    client.close()
    time.sleep(0.01)
    client.connect()
    time.sleep(0.1)
        # Print the value of the holding register
    print(f"Holding Register Value: {vl_feedback.registers[0]}")
    speedL = conv_msg(vl_feedback.registers[1], velocity_l)