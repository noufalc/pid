import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import time
import numpy as np
import os



class PID(object):

    '''Defines an object to implement the speed control of a DC motor.'''

    def __init__(self, K_p, K_i, K_d):
        self.speed = 0
        self.speed_count = 0
        self.interval_start_time = 0
        self.reference_speed = 0
        self.function_time = 0.0 # to be verified
        self.speed_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.X = np.array([0.0,0.0,0.0])
        self.sampling_time = 8e-3
        self.sampling_frequency = int(1/self.sampling_time)
        self.K_p = K_p #cl 0.0840 #p  0.0980  #s  0.0462 #n  0.0280 
        self.K_i = K_i #   0.0476 #   0.0695   #  0.0262 #   0.0159    
        self.K_d = K_d #   0.0370 #   0.0518   #  0.0543 #   0.0329    
        self.output_file = open('data.csv', 'w')
        self.pwm_frequency = 20000
        self.start_time = 0
        self.last_error = 0.0
        self.total_error = 0.0

    # lambda function definitions
    current_micro_time = lambda self: int(round(time.time() * 1000000))
    f_reference_accelaration = lambda self, t: 0.0*t*((t>0)&(t<=4))+(50.0*t - 200.0)*((t>4)&(t<=6))+(100.0)*((t>6)&(t<=12))+(-50.0*t+700.0)*((t>12)&(t<=14))+0.0*(t>14)


    def init_process(self):
        self.initial_cleanup()
        self.setup_adc()
        self.init_pwm()
        self.setup_gpio()
        self.set_interval_start_time()
        self.init_interrupts()
        self.start_time = self.current_micro_time()


    def init_interrupts(self):
        GPIO.add_event_detect("P9_12", GPIO.RISING, self.detection, 1)
        GPIO.add_event_detect("P9_13", GPIO.RISING, self.calculate, 1)
        

    def setup_gpio(self):
        print "Setting up GPIO ..."
        GPIO.setup("P9_12", GPIO.IN)
        GPIO.setup("P9_13", GPIO.IN)
        print "Pins P9_12 and P9_13 started as INPUT pins successfully"


    def init_pwm(self):
        print "Starting PWM at pin P9_14 with 50Hz, 50% duty cycle"
        PWM.start("P9_14", 50, self.sampling_frequency, 0)
        PWM.start("P9_21", 0, self.pwm_frequency, 0)
        print "PWM started successfully"


    def setup_adc(self):
        print "Setting up ADC..."
        print "Connect pin number P9_33 to voltage measurement"
        print "Connect pin number P9_34 to ground"
        ADC.setup()
        print "ADC setup success"


    def initial_cleanup(self):
        print "Clean up of PWM and GPIO..."
        print "Clean up success"


    def detection(self, args): 
         self.speed_count += 1


    def measure_voltage(self):
        adc_out = 0.3333 * (ADC.read("P9_33") +  ADC.read("P9_33") + ADC.read("P9_33"))
        return adc_out * 1.8 * 11.2461


    def set_interval_start_time(self):
        self.interval_start_time = self.current_micro_time() 


    def get_present_speed(self):
        if self.speed_count >= 1:
            end_time = self.current_micro_time()
            self.speed = 60.0 * 1000000.0 * self.speed_count / (end_time - self.interval_start_time)
            self.speed_list.pop(0)
            self.speed_list.append(self.speed)
            self.speed_count = 0
            self.interval_start_time = end_time
        if (self.current_micro_time() - self.interval_start_time) > 1000000:
            print 'Inside stop speed'
            self.speed = 0
            self.speed_list.pop(0)
            self.speed_list.append(self.speed)
        return sum(self.speed_list) / len(self.speed_list)


    def get_reference_speed(self):
        self.reference_speed +=  self.f_reference_accelaration(self.function_time) * (self.sampling_time)
        self.function_time += self.sampling_time
        return self.reference_speed


    def get_reference_accelaration(self):
        return self.f_reference_accelaration(self.function_time)
    

    def remove_interrupts(self):
        GPIO.remove_event_detect("P9_13") 
        GPIO.remove_event_detect("P9_12")       
    

    def stopPWM(self):
        PWM.cleanup();


    def stopProcess(self):
        self.remove_interrupts()
        self.output_file.close()
        PWM.cleanup()
        print 'Interrupts removed'
        self.output_file`
        print 'Process finished'

    def calculate(self, args):        
        speed = self.get_present_speed();
        reference_speed = self.get_reference_speed() 
        error = reference_speed - speed
        derror = error - self.last_error
        self.total_error += error * self.sampling_time
        control_duty = self.K_p * error  + (self.K_d * derror / self.sampling_time) + self.K_i * self.total_error
        self.output_file.write(str(self.function_time) + ' ' +  str(speed) + ' ' + str(reference_speed) + '\n')
        self.last_error = error
        if control_duty > 98.0:
            control_duty = 98.0
        if control_duty < 30.0:
            control_duty = 30.0
        PWM.set_duty_cycle("P9_21", control_duty)

if __name__ == "__main__":
    os.nice(10)
    my_pid= PID(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))
    my_pid.init_process()
    time.sleep(50)
    my_pid.stopProcess()
