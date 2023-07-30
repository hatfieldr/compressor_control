from ADCPi import ADCPi

class Pressure:
    def __init__(self,channels,offsets,scales):
        self.P[len(offsets)]
        self.adc = ADCPi(0x68, 0x69,12)
        self.offsets = offsets
        self.scales = scales
        self.channels = channels
    def read_presures(self):
        for index in range(0,self.channels):
            self.P[index]=self.adc.read_voltage(self.channels[index])
            self.P[index] = (self.P[index]-self.offsets[index])*self.scales[index]
    def get_pressure_string(self,index):
        #if pressure < -5, sensor or loop is faulty
        if self.P[index] < -5:
            return "P Sensor FAULTY"
        else:
            return f"Pressure: {self.P[index]:.1f} Bar"


     
if __name__ == '__main__':
    import time
    channels = range(1,5)
    press = pressure(channels=channels,offsets=[0.481845,0.481845,0.481845,0.481845],
                     scales=[25.941952,25.941952,25.941952,25.941952])
    try:
        while True:
            press.read_presures()
            for channel in channels:
                print(press.get_pressure_string(channel))  
            time.sleep(1)      
    except KeyboardInterrupt:
        pass
