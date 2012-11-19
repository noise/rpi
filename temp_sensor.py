#!/usr/bin/python
import math

'''
Code for reading data from a DS18B20 digital temp sensor via 1-wire from a Raspberry Pi. 
Includes a simple Kalman filter to smooth out the data.

See:
* http://www.picymru.com/633
* http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
'''

class KalmanFilter():
    '''
    shamelessly stolen from http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
    '''

    def __init__(self, q, r, p, init_val):
        self.q = q # process noise covariance
        self.r = r # measurement noise covariance
        self.p = p # estimation error covariance
        self.k = 0 # kalman gain
        self.x = init_val

    def update(self, measurement):
        #prediction update
        #omit x = x
        self.p = self.p + self.q;

        #measurement update
        self.k = self.p / (self.p + self.r);
        self.x = self.x + self.k * (measurement - self.x);
        self.p = (1 - self.k) * self.p;
        return self.x

    def value(self):
        return self.x
        
class Sensor():
    sensor_fn = "/sys/bus/w1/devices/w1 bus master/%s/w1_slave"
    MIN_VAL = 1.0
    MAX_VAL = 99.0
    ALLOWED_VARIANCE = 10.0 # max allowed diff in consecutive readings

    INIT_VAL = 25.0

    def __init__(self, id):
        self.id = id
        self.val = self.INIT_VAL
        self.lastVal = self.INIT_VAL
        self.filter = KalmanFilter(0.5, 5.0, 5.0, self.INIT_VAL) # low process noise, high sensor noise

    def read(self):
        sensor_fn_id = self.sensor_fn % self.id
        sensor_f = open(sensor_fn_id)
        data = sensor_f.read() 
        sensor_f.close() 
        secondline = data.split("\n")[1] 
        data = secondline.split(" ")[9] 
        c = float(data[2:]) 
        c = c / 1000 

        if (c < self.MIN_VAL or c > self.MAX_VAL) or (math.fabs(self.lastVal - c) > self.ALLOWED_VARIANCE):
            #print "bad data: %f" % c
            pass
        else:
            self.val = c
            self.lastVal = self.val
            self.filter.update(self.val)

    def value(self):
        return self.filter.value()

    def rawValue(self):
        return self.val

    def __repr__(self):
        return 'id: %s, raw: %f, filtered: %f' % (self.id, self.rawValue(), self.value())

    def c2f(self, c):
        return c * 9.0/5.0 + 32.0

class Sensors():

    sensors_fn = "/sys/bus/w1/devices/w1 bus master/w1_master_slaves"

    sensors = []

    def __init__(self):
        sensors_f = open(self.sensors_fn)
        sensors = sensors_f.read()
        sensors_f.close()
        for sensor_id in sensors.split('\n'):
            if sensor_id == '':
                continue
            self.sensors.append(Sensor(sensor_id))

    def readAll(self):
        for sensor in self.sensors:
            sensor.read()
            
    def __repr__(self):
        return str(self.sensors)



if __name__ == '__main__':
    sensors = Sensors()
    while True:
        sensors.readAll()
        print sensors 


