from ADCPi import ADCPi
import adafruit_mcp9600
import board
import busio
import numpy as np

class Pressure:
    def __init__(self,channels,offsets,scales):
        self.P = np.empty(1,len(offsets))
        self.adc = ADCPi(0x68, 0x69,12)
        self.offsets = offsets
        self.scales = scales
        self.channels = channels
    def read(self):
        for index,channel in enumerate(self.channels):
            self.P[index]=self.adc.read_voltage(channel)
            self.P[index] = (self.P[index]-self.offsets[index])*self.scales[index]
    def get_string(self,index):
        #if pressure < -5, sensor or loop is faulty
        if self.P[index] < -5:
            return "P Sensor FAULTY"
        else:
            return f"Pressure: {self.P[index]:.1f} Bar"

class Temperature:
    def __init__(self,addresses):
        self.addresses = addresses
        self.T = self.P = np.empty(1,len(addresses))
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    def read(self):
        for idx, address in enumerate(self.addresses):
            self.T[idx] = adafruit_mcp9600.MCP9600(self.i2c,address=address).temperature
    
    def getstring(self,index):
        return f'Temp: {self.T[index]} \u00b0C'


     
if __name__ == '__main__':
    import time
    channels = range(1,5)
    addresses=[0x66,0x65,0x64]
    press = Pressure(channels=channels,offsets=[0.481845,0.481845,0.481845,0.481845],
                     scales=[25.941952,25.941952,25.941952,25.941952])
    temp = Temperature(addresses=addresses)
    try:
        while True:
            press.read()
            temp.read()
            for idx,channel in enumerate(channels):
                print(f'Pressure Channel: {channel} ={press.get_string(idx)}')  
            for idx,address in enumerate(addresses):
                print(f'Temp Address: {address} ={temp.get_string(idx)}')  
            time.sleep(1)      
    except KeyboardInterrupt:
        pass
