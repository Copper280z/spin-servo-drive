import matplotlib.pyplot as plt
import serial, time
import simplefoc, simplefoc.packets
from rx import operators as ops
from simplefoc import registers

import numpy as np

def to_numpy(data):
    datavals = []
    total_samples = 0
    for thing in data:
        # print(type(thing))
        if type(thing) is list:
            # print(len(thing))
            total_samples += len(thing)
            for frame in thing:
                datavals.append(frame.values)
    values = np.asarray(datavals)
    return values

def get_data(motor,tele,callback):
    data = []
    def append_data(thing):
        data.append(thing)

    frames = tele.observable().pipe(ops.buffer_with_time(1.0)).subscribe(append_data)
    tele.set_downsample(1)
    callback(motor)
    tele.set_downsample(0)
    time.sleep(3)
    ret = to_numpy(data)
    frames.dispose()
    return ret

registers.SimpleFOCRegisters.add_register("REG_CURRENT_SP",0x90,['f'],['f'])
registers.SimpleFOCRegisters.add_register("REG_VOLTAGE_QFF",0x91,['f'],['f'])
registers.SimpleFOCRegisters.add_register("REG_VOLTAGE_DFF",0x92,['f'],['f'])


PORT="/dev/cu.usbmodem2086308E484E1"
BAUD=921600*2

# assume registers and downsample=0 set in arduino code
sf = simplefoc.packets.serial(PORT, BAUD) #, simplefoc.packets.ProtocolType.ascii)
motor = sf.motor(1)
sf.connect()
tele = sf.telemetry()



motor.set_limits(max_current=3)
motor.set_angle_pid(p=0.5,i=0.5,d=0.001)
#%%

def action(motor):
    time.sleep(0.05)
    motor.set_target(6.28)
    time.sleep(0.5)
    motor.set_target(0)
    time.sleep(0.5)

motor.set_target(0)

time.sleep(0.5)

data = get_data(motor,tele,action)

time_est = np.arange(data.shape[0])/12000

plt.figure()
plt.subplot(311)
plt.plot(time_est,data[:,0])
plt.plot(time_est,data[:,1])
plt.subplot(312)
plt.plot(time_est,data[:,4])
plt.plot(time_est,data[:,3])
plt.subplot(313)
# plt.plot(time_est,data[:,2])
plt.plot(time_est,data[:,5])
plt.plot(time_est,data[:,6])
plt.show()

plt.figure()
plt.plot(time_est,data[:,6]/data[:,3])

print(f'got {data.shape[0]} datapoints for {data.shape[1]} variables')
