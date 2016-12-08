import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import time
import numpy as np
import os



class ADRC(object):

    '''Defines an ADRC object to implement the speed control of a DC motor.
       No need to pass any arguments'''

    def __init__(self):
        self.speed = 0
        self.speed_count = 0
        self.interval_start_time = 0
        self.reference_speed = 0
        self.function_time = 0.0 # to be verified
        self.speed_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.f =  np.array([
                [   0.95276121411048309006730505643645,    0.007810034411958742713022463277639, 0.000031492074241769127558848023751281],
                [ -0.093972349537439062161503500192339,     0.99962142058223557583573892770801,   0.0079989868574093583863948708767566],
                [  -0.06248027529566994864307361012834, -0.00025193659393415296626067556573503,     0.99999932547313674646716208371799]
                ])      
 
        self.h =  np.array([
                [    0.000031492074241769120782584445716878,  0.04723878588951693768827055919246],
                [      0.0079989868574093566516713948999495, 0.093972349537439256450532809594733],
                [ -0.00000067452686319471487005894339006318, 0.062480275295670739676978655552375]
                ])


        self.X = np.array([0.0,0.0,0.0])
        self.sampling_time = 8e-3
        self.sampling_frequency = int(1/self.sampling_time)
        self.K_p = 0.0280   # 0.0462   # 0.0980   #  0.0840
        self.K_d = 0.0329   # 0.0543   # 0.0518   # 0.0370
        self.K_i = 0.0159   # 0.0262   # 0.0695   # 0.0476
        self.b = 0.2
        self.output_file = open('data.csv', 'w')
        self.pwm_frequency = 20000
        self.start = 0
        self.last_error = 0.0
        self.total_error = 0.0

    # lambda function definitions
    current_micro_time = lambda self: int(round(time.time() * 1000000))
    f_reference_accelaration = lambda self, t: 240.0*t*((t>0)&(t<=1))+(240.0)*((t>1)&(t<=3))+(-240.0*t+960.0)*((t>3)&(t<=4))+0.0*(t>4)
    f_reference_speed = lambda self, t: 0.0*t*((t>0)&(t<=10))+(600.0)*(t>10)


    def init_process(self):
        self.initial_cleanup()
        self.setup_adc()
        self.init_pwm()
        self.setup_gpio()
        self.set_interval_start_time()
        self.init_interrupts()
        self.start = self.current_micro_time()


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
        self.function_time += self.sampling_time
        return self.f_reference_speed(self.function_time)


    def get_reference_accelaration(self):
        return self.f_reference_accelaration(self.function_time)
    

    def remove_interrupts(self):
        GPIO.remove_event_detect("P9_13") 
        GPIO.remove_event_detect("P9_12")       
    

    def stopPWM(self):
        PWM.cleanup();


    def calculate(self, args):        
        t = self.current_micro_time()
        speed = self.get_present_speed();
        reference_speed = self.get_reference_speed() 
        error = reference_speed - speed
        derror = error - self.last_error
        print error
        self.total_error += error * self.sampling_time
        control_duty = self.K_p * error  + (self.K_d * derror / self.sampling_time) + self.K_i * self.total_error
        #print "%.5f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t" %(voltage, speed, reference_speed,  X_updated[0], X_updated[1], control_duty),
        self.output_file.write(str(self.function_time) + ' ' +  str(speed) + ' ' + str(reference_speed) + '\n')
        #self.output_file.write("%.5f\t%.5f\t%.5f\n" %(self.function_time, reference_speed, reference_accelaration))
        #print ' ', control_duty
        self.last_error = error
        if control_duty > 98.0:
            control_duty = 98.0
        if control_duty < 30.0:
            control_duty = 30.0
        PWM.set_duty_cycle("P9_21", control_duty)
        #print self.current_micro_time() - t

if __name__ == "__main__":
    os.nice(10)
    my_adrc = ADRC()
    my_adrc.init_process()
    time.sleep(70)
    my_adrc.remove_interrupts()
    my_adrc.stopPWM()
    print 'Interrupts removed'
    print 'Process finished'
    PWM.cleanup()
