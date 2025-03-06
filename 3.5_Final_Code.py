from machine import Pin, PWM 
import time

Dir_forward = 1
Dir_backward = -1

LF_forward = 1
LF_backward = -1
LB_forward = 1
LB_backward = -1
RF_forward = 1
RF_backward = -1
RB_forward = 1
RB_backward = -1

# mapping function
def map(x,in_max, in_min, out_max, out_min):
    return (x - in_min)/(in_max - in_min)*(out_max - out_min) + out_min

'''
Motor interface.
    M1  _____  M2
       |     |
       |     |
       |     |
    M4 |_____| M3
'''
# left front wheel M1
Motor_LF_PWM = PWM(Pin(12))
Motor_LF_Dir = Pin(13, Pin.OUT)
# right front wheel M2
Motor_RF_PWM = PWM(Pin(15))
Motor_RF_Dir = Pin(14, Pin.OUT)
# right back wheel M3
Motor_RB_PWM = PWM(Pin(16))
Motor_RB_Dir = Pin(17, Pin.OUT)
# left back wheel M4
Motor_LB_PWM = PWM(Pin(19))
Motor_LB_Dir = Pin(18, Pin.OUT)

class  Motor():
    def __init__(self):
        pass
    
    '''
    (1, 1, 100) ---(status, direction, speed)
     |  |   |
     |  |   speed: value range 0~100.
     |  direction: 1(forward), -1(backward)
    status: 1(move), 0(stop)   
    '''
    # Control the M1 motor (left front).
    def motor_left_front(self, status, direction, speed):
        '''
        if status == 0: # stop.
            Motor_LF_Dir.low()
            Motor_LF_PWM.duty_u16(0)
        else:
            value = int(map(speed,100,0,65535,0))
            Motor_LF_PWM.freq(500)
            if direction == Dir_forward: # 1, motor forward.
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)
            elif direction == Dir_backward: # -1, motor backward.
                Motor_LF_Dir.high()
                Motor_LF_PWM.duty_u16(65535-value)
        '''
        
        if status == 0: # stop.
            Motor_LF_PWM = PWM(Pin(12))
            Motor_LF_Dir = Pin(13, Pin.OUT)
            Motor_LF_Dir.low()
            Motor_LF_PWM.duty_u16(0)
        else:
            value = int(map(speed,100,0,65535,0))
            
            if direction == Dir_forward:
                
                Motor_LF_PWM = PWM(Pin(12))
                Motor_LF_Dir = Pin(13, Pin.OUT)
                Motor_LF_PWM.freq(500)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_LF_PWM = PWM(Pin(13))
                Motor_LF_Dir = Pin(12, Pin.OUT)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)
    
    # Control the M2 motor (right front).
    def motor_right_front(self, status, direction, speed):
        '''
        if status == 0: # stop.
            Motor_LB_Dir.low()
            Motor_LB_PWM.duty_u16(0)
        else:
            value = int(map(speed,0,100,0,65535))
            Motor_LB_PWM.freq(500)
            if direction == Dir_forward:
                Motor_LB_Dir.high()
                Motor_LB_PWM.duty_u16(65535-value)
            elif direction == Dir_backward:
                Motor_LB_Dir.low()
                Motor_LB_PWM.duty_u16(value)
        '''
        
        if status == 0: # stop.
            Motor_RF_PWM = PWM(Pin(15))
            Motor_RF_Dir = Pin(14, Pin.OUT)
            Motor_RF_Dir.low()
            Motor_RF_PWM.duty_u16(0)
        else:
            value = int(map(speed,100,0,65535,0))
            
            if direction == Dir_forward:
                
                Motor_RF_PWM = PWM(Pin(15))
                Motor_RF_Dir = Pin(14, Pin.OUT)
                Motor_RF_PWM.freq(500)
                Motor_RF_Dir.low()
                Motor_RF_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_RF_PWM = PWM(Pin(14))
                Motor_RF_Dir = Pin(15, Pin.OUT)
                Motor_RF_Dir.low()
                Motor_RF_PWM.duty_u16(value)
    
    # Control the M3 motor (right back).
    def motor_right_back(self, status, direction, speed):
        '''
        if status == 0: # stop.
            Motor_RF_Dir.low()
            Motor_RF_PWM.duty_u16(0)
        else:
            value = int(map(speed,0,100,0,65535))
            Motor_RF_PWM.freq(500)
            if direction == Dir_forward:
                Motor_RF_Dir.low()
                Motor_RF_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_RF_Dir.high()
                Motor_RF_PWM.duty_u16(65535-value)
        '''
        
        if status == 0: # stop.
            Motor_RB_PWM = PWM(Pin(16))
            Motor_RB_Dir = Pin(17, Pin.OUT)
            Motor_RB_Dir.low()
            Motor_RB_PWM.duty_u16(0)
        else:
            value = int(map(speed,100,0,65535,0))
            
            if direction == Dir_forward:
                
                Motor_RB_PWM = PWM(Pin(17))
                Motor_RB_Dir = Pin(16, Pin.OUT)
                Motor_RB_PWM.freq(500)
                Motor_RB_Dir.low()
                Motor_RB_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_RB_PWM = PWM(Pin(16))
                Motor_RB_Dir = Pin(17, Pin.OUT)
                Motor_RB_Dir.low()
                Motor_RB_PWM.duty_u16(value)
    
    # Control the M4 motor (left back).
    def motor_left_back(self, status, direction, speed):
        '''
        if status == 0: # stop.
            Motor_RB_Dir.low()
            Motor_RB_PWM.duty_u16(0)
        else:
            value = int(map(speed,0,100,0,65535))
            Motor_RB_PWM.freq(500)
            if direction == Dir_forward:
                Motor_RB_Dir.high()
                Motor_RB_PWM.duty_u16(65535-value)
            elif direction == Dir_backward:
                Motor_RB_Dir.low()
                Motor_RB_PWM.duty_u16(value)
            '''
        
        if status == 0: # stop.
            Motor_LB_PWM = PWM(Pin(19))
            Motor_LB_Dir = Pin(18, Pin.OUT)
            Motor_LB_Dir.low()
            Motor_LB_PWM.duty_u16(0)
        else:
            value = int(map(speed,100,0,65535,0))
            
            if direction == Dir_forward:
                
                Motor_LB_PWM = PWM(Pin(18))
                Motor_LB_Dir = Pin(19, Pin.OUT)
                Motor_LB_PWM.freq(500)
                Motor_LB_Dir.low()
                Motor_LB_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_LB_PWM = PWM(Pin(19))
                Motor_LB_Dir = Pin(18, Pin.OUT)
                Motor_LB_Dir.low()
                Motor_LB_PWM.duty_u16(value)
 
                
    def motor_stop(self):
            Motor_LF_PWM = PWM(Pin(12))
            Motor_LF_Dir = Pin(13, Pin.OUT)
            Motor_RF_PWM = PWM(Pin(16))
            Motor_RF_Dir = Pin(17, Pin.OUT)
            Motor_RB_PWM = PWM(Pin(19))
            Motor_RB_Dir = Pin(18, Pin.OUT)
            Motor_LB_PWM = PWM(Pin(15))
            Motor_LB_Dir = Pin(14, Pin.OUT)
            
            Motor_LF_Dir.low()
            Motor_LF_PWM.duty_u16(0)
            Motor_LB_Dir.low()
            Motor_LB_PWM.duty_u16(0)
            Motor_RF_Dir.low()
            Motor_RF_PWM.duty_u16(0)
            Motor_RB_Dir.low()
            Motor_RB_PWM.duty_u16(0)
        
    def move(self, status, direction, speed):
        if status == 0:
            self.motor_stop()
        else:
            if direction == "forward":
                print('forward')
                self.motor_left_front(1, LF_forward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)   # M3
                self.motor_left_back(1, LB_forward, speed)    # M4
            elif direction == "backward":
                print('backward')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_backward, speed)   # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
            elif direction == "left":
                print('left')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)   # M2
                self.motor_right_back(1, RB_backward, speed)   # M3
                self.motor_left_back(1, LB_forward, speed)     # M4
            elif direction == "left":
                print('left')
                self.motor_left_front(1, LF_forward, speed)    # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)    # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
                
            elif direction == "left_forward":
                print('left_forward')
                self.motor_left_front(0, LF_forward, speed*0.5)   # M1
                self.motor_right_front(1, RF_forward, speed)      # M2
                self.motor_right_back(0, RB_forward, speed*0.5)   # M3
                self.motor_left_back(1, LB_forward, speed)        # M4
                
            elif direction == "right_forward":
                print('right_forward')
                self.motor_left_front(1, LF_forward, speed)       # M1
                self.motor_right_front(0, RF_forward, speed*0.5)  # M2
                self.motor_right_back(1, RB_forward, speed)       # M3
                self.motor_left_back(0, LB_forward, speed*0.5)    # M4
            elif direction == "left_backward":
                print('left_backward')
                self.motor_left_front(1, LF_backward, speed)       # M1
                self.motor_right_front(0, RF_backward, speed*0.5)  # M2
                self.motor_right_back(1, RB_backward, speed)       # M3
                self.motor_left_back(0, LB_backward, speed*0.5)    # M4
                
            elif direction == "right_backward":
                print('right_backward')
                self.motor_left_front(0, LF_backward, speed*0.5)   # M1
                self.motor_right_front(1, RF_backward, speed)      # M2
                self.motor_right_back(0, RB_backward, speed*0.5)   # M3
                self.motor_left_back(1, LB_backward, speed)        # M4
                
            elif direction == "turn_left":
                print('turn_left')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)   # M2
                self.motor_right_back(1, RB_forward, speed)    # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
                
            elif direction == "turn_right":
                print('turn_right')
                self.motor_left_front(1, LF_forward, speed)     # M1
                self.motor_right_front(1, RF_backward, speed)   # M2
                self.motor_right_back(1, RB_backward, speed)    # M3
                self.motor_left_back(1, LB_forward, speed)      # M4
            else:
                print("Direction error!")
          
                   
    def motor_left_front11(self, status, direction, speed):
        
        if status == 0: # stop.
            Motor_LF_PWM = PWM(Pin(12))
            Motor_LF_Dir = Pin(13, Pin.OUT)
            Motor_LF_Dir.low()
            Motor_LF_PWM.duty_u16(0)
        else:
            value = int(map(speed,100,0,65535,0))
            
            if direction == Dir_forward:
                
                Motor_LF_PWM = PWM(Pin(12))
                Motor_LF_Dir = Pin(13, Pin.OUT)
                Motor_LF_PWM.freq(500)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)
            elif direction == Dir_backward:
                Motor_LF_PWM = PWM(Pin(13))
                Motor_LF_Dir = Pin(12, Pin.OUT)
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(value)
    def test(self):  # test M1 motor.
        print("test")
        
        #self.motor_left_front(1, 1, 100)
        #time.sleep(2)
        #self.motor_left_front(1, -1, 100)
        #time.sleep(2)
        self.move(status=1, direction="forward", speed=100)
        #self.move(1, "turn_right", 80)
        #self.move(1, "backward", 80)
        #self.move(1, "left_backward", 50)
        time.sleep(10)
        
if __name__ == '__main__':
    motor = Motor()
    try:
        motor.test()
        motor.motor_stop()
        
        #motor.motor_left_front(1, 1, 100)
        '''
        (1, 1, 100) ---(status, direction, speed)
         |  |   |
         |  |   speed: value range 0~100.
         |  direction: 1(forward), -1(backward)
        status: 1(move), 0(stop)   
        '''
        #motor.motor_left_front(1, 1, 100)
        #motor.motor_left_back(1, 1, 100)
        #motor.motor_right_front(1, 1, 100)
        #motor.motor_right_back(1, 1, 100)
        #time.sleep(10)
    
    except KeyboardInterrupt:
        motor.motor_stop() # motor stop.

