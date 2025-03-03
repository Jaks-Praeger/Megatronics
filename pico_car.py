from machine import Pin, PWM, I2C
import array, time
import rp2
import os
import machine
from time import sleep_ms


import micropython
micropython.alloc_emergency_exception_buf(100)

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

Adjustment = 1

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

# mapping function
def map(x,in_max, in_min, out_max, out_min):
    return (x - in_min)/(in_max - in_min)*(out_max - out_min) + out_min

class Buzzer():
    def __init__(self):
        self._buzzer = PWM(Pin(26, Pin.OUT))

    def playtone(self, frequency):
        self._buzzer.duty_u16(60000)
        self._buzzer.freq(frequency)
        
    def sound(self):
        buzzer.playtone(1500)
        

    def bequiet(self):
        self._buzzer.duty_u16(0)
        
class Servo():
    pwm_max = 2500
    pwm_min = 500
    period = 65535 # oxFFFF
    
    def __init__(self):
        pass
    
    
    def  set_angle(self, pin, angle):  # Pin:Servo GPIO pins, angle:Servo rotation angle -90~90.
        self.servo = PWM(Pin(pin, Pin.OUT))
        self.servo.freq(50) # Set servo Freq.
        if angle < -90:
            angle = 90
        if angle > 90:
            angle = 90
        high_level_time = map(angle, 90, -90, self.pwm_max, self.pwm_min)
        # Servo duty cycle value.
        duty_cycle_value = int((high_level_time/20000)*self.period)
        self.servo.duty_u16(duty_cycle_value)
        
class Ultrasonic():
    # Define output(trig) and input(echo) pins.
    def __init__(self, trig, echo):
        self._trig = Pin(trig, Pin.OUT)
        self._echo = Pin(echo, Pin.IN)
    
    def get_distance(self):
        # Generate 10us square wave.
        self._trig.low()
        time.sleep_us(2)
        self._trig.high()
        time.sleep_us(10)
        self._trig.low()
        self.start = 0
        self.end = 0
        while self._echo.value() == 0:
            self.start = time.ticks_us()
        while self._echo.value() == 1:
            self.end = time.ticks_us()
        dis = (self.end - self.start) * 0.0343 / 2
        # round to two decimal places.
        return round(dis, 2)
    

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812_led():
    T1 = 2
    T2 = 5
    T3 = 1
    
    wrap_target() # .side()Specifies where the program continues execution.
    label("bitloop")
    out(x, 1).side(0)[T3 - 1] # []Delay a certain number of cycles after executing the instruction.
    jmp(not_x, "do_zero").side(1)[T1 - 1]
    jmp("bitloop").side(1)[T2 - 1]
    label("do_zero")
    nop().side(0)[T2 - 1]
    wrap()
 
class WS2812():
    def __init__(self):
        # Configure the number of WS2812 LEDs, pins and brightness.
        # 配置 WS2812/SK6812(color_led) LED 的数量、引脚和亮度。
        self._led_num = 4 #  LED number
        self._pin = 11    # GPIO11
        self.brightness_value = 0.2  # light brightness. The value range is 0~1.
        # Create the StateMachine with the ws2812 program, outputting on Pin(PIN_NUM).
        self.sm = rp2.StateMachine(0, ws2812_led, freq=8_000_000, sideset_base=Pin(self._pin))
        # Start the StateMachine, it will wait for data on its FIFO.
        self.sm.active(1)
        # Display a pattern on the LEDs via an array of LED RGB values.
        self.ar = array.array("I", [0 for _ in range(self._led_num)])
        
    def pixels_show(self):
        dimmer_ar = array.array("I", [0 for _ in range(self._led_num)])
        for i,c in enumerate(self.ar):
            r = int(((c >> 8) & 0xFF))
            g = int(((c >> 16) & 0xFF))
            b = int((c & 0xFF))
            dimmer_ar[i] = (g<<16) + (r<<8) + b
        self.sm.put(dimmer_ar, 8)
        time.sleep_ms(10)

    # Set multiple colors
    # Set the color of the specified ws2812.
    def pixels_set(self,i, color):
        R = round(color[0] * self.brightness_value)
        G = round(color[1] * self.brightness_value)
        B = round(color[2] * self.brightness_value)
        self.ar[i] = (G<<16) + (R<<8) + B
        self.pixels_show()
        
    # Set the color of all ws2812 led.
    def pixels_fill(self,color):
        for i in range(len(self.ar)):
            self.pixels_set(i, color)
    
    # Set a single color.
    def set_color(self,i,R,G,B):
        
        R = round(R * self.brightness_value)
        G = round(G * self.brightness_value)
        B = round(B * self.brightness_value)
        self.ar[i] = (G<<16) + (R<<8) + B
        self.pixels_show()
    
    def set_color_all(self, R, G, B):
        for i in range(len(self.ar)):
            self.set_color(i, R, G, B)
    
    # Adjust the brightness, the value range is 0-1, the default value is 0.2
    def brightness(self, brightness = None):
        if brightness == None:
            return self.brightness_value
        else:
            if (brightness < 0):
                brightness = 0
        if (brightness > 1):
            brightness = 1
        self.brightness_value = brightness
        
    # breathing light.
    def breath(self, R, G, B):
        breathSteps = 10
        for i in range(1,breathSteps):
            Breath_R = round(R*i/breathSteps)
            Breath_G = round(G*i/breathSteps)
            Breath_B = round(B*i/breathSteps)
            self.set_color_all(Breath_R, Breath_G, Breath_B)
            
            #self.pixels_show()
            #self.setColor(self.colorBreathR*i/self.breathSteps, self.colorBreathG*i/self.breathSteps, self.colorBreathB*i/self.breathSteps)
            time.sleep(0.06)
        for i in range(1,breathSteps):
            Breath_R = round(R-R*i/breathSteps)
            Breath_G = round(G-G*i/breathSteps)
            Breath_B = round(B-B*i/breathSteps)
            self.set_color_all(Breath_R, Breath_G, Breath_B)
            #self.pixels_show()
            
            #self.setColor(self.colorBreathR-(self.colorBreathR*i/self.breathSteps), self.colorBreathG-(self.colorBreathG*i/self.breathSteps), self.colorBreathB-(self.colorBreathB*i/self.breathSteps))
            time.sleep(0.06)
    
    # Lights off
    def stop(self):
        robot_light.set_color_all(0,0,0)
        

class Motor():
    def __init__(self):
        pass
    
    '''
    (1, 1, 100) ---(status, direction, speed)
     |  |   |
     |  |   speed: value range 0~100.
     |  direction: 1(forward), -1(backward)
    status: 1(move), 0(stop)   
    '''
    '''
    # Control the M1 motor (left front).
    def motor_left_front(self, status, direction, speed):
        if status == 0: # stop.
            Motor_LF_Dir.low()
            Motor_LF_PWM.duty_u16(0)
        else:
            value = int(map(speed,100,0,65535,0))
            Motor_LF_PWM.freq(500)
            if direction == Dir_forward: # 1, motor forward.
                Motor_LF_Dir.low()
                Motor_LF_PWM.duty_u16(int(value*Adjustment))
            elif direction == Dir_backward: # -1, motor backward.
                Motor_LF_Dir.high()
                Motor_LF_PWM.duty_u16(65535-value)
    
    # Control the M2 motor (left back).
    def motor_left_back(self, status, direction, speed):
        value = 0.9
        if status == 0: # stop.
            Motor_LB_Dir.low()
            Motor_LB_PWM.duty_u16(0)
        else:
            value = int(map(speed,0,100,0,65535))
            Motor_LB_PWM.freq(500)
            if direction == Dir_forward:
                Motor_LB_Dir.high()
                Motor_LB_PWM.duty_u16(int((65535-value)*Adjustment))
            elif direction == Dir_backward:
                Motor_LB_Dir.low()
                Motor_LB_PWM.duty_u16(value)
    
    # Control the M3 motor (right front).
    def motor_right_front(self, status, direction, speed):
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
                Motor_RF_PWM.duty_u16(int((65535-value)*Adjustment))
    
    # Control the M4 motor (right back).
    def motor_right_back(self, status, direction, speed):
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
                Motor_RB_PWM.duty_u16(int(value*Adjustment))
                
    def motor_right_back11(self, status, direction, speed):
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
                
    def motor_stop(self):
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
                #print('forward')
                self.motor_left_front(1, LF_forward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)   # M3
                self.motor_left_back(1, LB_forward, speed)    # M4
            elif direction == "backward":
                #print('backward')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_backward, speed)   # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
            elif direction == "right":
                #print('right')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)   # M2
                self.motor_right_back(1, RB_backward, speed)   # M3
                self.motor_left_back(1, LB_forward, speed)     # M4
            elif direction == "left":
                #print('left')
                self.motor_left_front(1, LF_forward, speed)    # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)    # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
                
            elif direction == "right_forward":
                #print('right_forward')
                self.motor_left_front(0, LF_forward, speed*0.5)   # M1
                self.motor_right_front(1, RF_forward, speed)      # M2
                self.motor_right_back(0, RB_forward, speed*0.5)   # M3
                self.motor_left_back(1, LB_forward, speed)        # M4
                
            elif direction == "left_forward":
                #print('left_forward')
                self.motor_left_front(1, LF_forward, speed)       # M1
                self.motor_right_front(0, RF_forward, speed*0.5)  # M2
                self.motor_right_back(1, RB_forward, speed)       # M3
                self.motor_left_back(0, LB_forward, speed*0.5)    # M4
            elif direction == "right_backward":
                #print('right_backward')
                self.motor_left_front(1, LF_backward, speed)       # M1
                self.motor_right_front(0, RF_backward, speed*0.5)  # M2
                self.motor_right_back(1, RB_backward, speed)       # M3
                self.motor_left_back(0, LB_backward, speed*0.5)    # M4
                
            elif direction == "left_backward":
                #print('left_backward')
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
        '''
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
                #print('forward')
                self.motor_left_front(1, LF_forward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)   # M3
                self.motor_left_back(1, LB_forward, speed)    # M4
            elif direction == "backward":
                #print('backward')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_backward, speed)   # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
            elif direction == "right":
                #print('right')
                self.motor_left_front(1, LF_forward, speed)    # M1
                self.motor_right_front(1, RF_backward, speed)  # M2
                self.motor_right_back(1, RB_forward, speed)    # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
            elif direction == "left":
                #print('left')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)   # M2
                self.motor_right_back(1, RB_backward, speed)   # M3
                self.motor_left_back(1, LB_forward, speed)     # M4
                
            elif direction == "left_forward":
                #print('left_forward')
                self.motor_left_front(0, LF_forward, speed*0.5)   # M1
                self.motor_right_front(1, RF_forward, speed)      # M2
                self.motor_right_back(0, RB_forward, speed*0.5)   # M3
                self.motor_left_back(1, LB_forward, speed)        # M4
                
            elif direction == "right_forward":
                #print('right_forward')
                self.motor_left_front(1, LF_forward, speed)       # M1
                self.motor_right_front(0, RF_forward, speed*0.5)  # M2
                self.motor_right_back(1, RB_forward, speed)       # M3
                self.motor_left_back(0, LB_forward, speed*0.5)    # M4
            elif direction == "left_backward":
                #print('left_backward')
                self.motor_left_front(1, LF_backward, speed)       # M1
                self.motor_right_front(0, RF_backward, speed*0.5)  # M2
                self.motor_right_back(1, RB_backward, speed)       # M3
                self.motor_left_back(0, LB_backward, speed*0.5)    # M4
                
            elif direction == "right_backward":
                #print('right_backward')
                self.motor_left_front(0, LF_backward, speed*0.5)   # M1
                self.motor_right_front(1, RF_backward, speed)      # M2
                self.motor_right_back(0, RB_backward, speed*0.5)   # M3
                self.motor_left_back(1, LB_backward, speed)        # M4
                
            elif direction == "turn_left":
                #print('turn_left')
                self.motor_left_front(1, LF_backward, speed)   # M1
                self.motor_right_front(1, RF_forward, speed)   # M2
                self.motor_right_back(1, RB_forward, speed)    # M3
                self.motor_left_back(1, LB_backward, speed)    # M4
                
            elif direction == "turn_right":
                #print('turn_right')
                self.motor_left_front(1, LF_forward, speed)     # M1
                self.motor_right_front(1, RF_backward, speed)   # M2
                self.motor_right_back(1, RB_backward, speed)    # M3
                self.motor_left_back(1, LB_forward, speed)      # M4
            else:
                print("Direction error!")
          
        
class Line_tracking():
    def __init__(self):
        self.ir_left = Pin(6, Pin.IN)
        self.ir_middle = Pin(5, Pin.IN)
        self.ir_right = Pin(4, Pin.IN)
        
    def get_ir_value(self):
        return [self.ir_left.value(),self.ir_middle.value() ,self.ir_right.value()]
    

'''
no repetition：
[9047, 4488, 554, 573, 558, 626, 556, 599, 558, 598, 558, 599, 557, 600, 532, 625, 532, 624, 558, 1672, 582, 1624, 554, 1698, 534, 1693, 533, 1697, 558, 1648, 555, 1696, 559, 1670, 557, 1671, 558, 598, 559, 1670, 558, 599, 582, 550, 582, 598, 559, 1671, 532, 623, 557, 600, 586, 1614, 562, 624, 557, 1645, 584, 1672, 558, 1671, 559, 573, 557, 1696, 557]
repetitions
[9072, 4492, 535, 601, 535, 600, 581, 571, 588, 596, 559, 572, 586, 596, 535, 622, 534, 597, 586, 1669, 581, 1669, 535, 1696, 585, 1644, 534, 1696, 559, 1670, 534, 1696, 558, 1673, 559, 1645, 610, 572, 560, 1670, 534, 629, 554, 570, 587, 573, 558, 1696, 559, 598, 533, 624, 558, 1671, 533, 623, 560, 1643, 588, 1642, 561, 1696, 533, 623, 560, 1671, 559, 39230, 9049, 2246, 583, 95490, 9027, 2265, 534, 95509, 9023, 2268, 559, 95482, 9014, 2269, 533, 95514, 9019, 2268, 585, 95443, 9041, 2268, 559, 95467, 9043, 2271, 531, 95495, 9043, 2247, 581, 95490, 9020, 2243, 559, 95517, 9043, 2246, 558, 95492, 9042, 2244, 559, 95492, 9017, 2268, 534, 
95490, 9046, 2269, 537, 95516, 9041, 2243, 535, 95517, 9089, 2152, 579, 95489, 9044, 2244, 585, 95492, 9025, 2269, 585]
'''
class IR(object):
    CODE = {162: "1", 98: "2", 226: "3", 34: "4", 2: "5", 194: "6", 224: "7", 168: "8", 144: "9",
            152: "0", 104: "*", 176: "#", 24: "up", 74: "down", 16: "left", 90: "right", 56: "ok"}

    def __init__(self, gpioNum):
        self.irRecv = machine.Pin(gpioNum, machine.Pin.IN, machine.Pin.PULL_UP)
        self.irRecv.irq(
             trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
             handler=self.__logHandler)

        self.ir_step = 0
        self.ir_count = 0
        self.buf64 = [0 for i in range(64)]
        self.recived_ok = False
        self.cmd = None
        self.cmd_last = None
        self.repeat = 0
        self.repeat_last = None
        self.t_ok = None
        self.t_ok_last = None
        self.start = 0
        self.start_last = 0        
        self.changed = False

    def __logHandler(self, source):
        thisComeInTime = time.ticks_us()

        # update time
        curtime = time.ticks_diff(thisComeInTime, self.start)
        self.start = thisComeInTime
        

        if curtime >= 8500 and curtime <= 9500:
            self.ir_step = 1
            return

        if self.ir_step == 1:
            if curtime >= 4000 and curtime <= 5000:
                self.ir_step = 2
                self.recived_ok = False
                self.ir_count = 0
                self.repeat = 0
            elif curtime >= 2000 and curtime <= 3000:  # Long press to repeat
                self.ir_step = 3
                self.repeat += 1

        elif self.ir_step == 2:  # receive 4 bytes
            self.buf64[self.ir_count] = curtime
            self.ir_count += 1
            if self.ir_count >= 64:
                self.recived_ok = True
                self.t_ok = self.start #Record the last ok time
                self.ir_step = 0

        elif self.ir_step == 3:  # repeat
            if curtime >= 500 and curtime <= 650:
                self.repeat += 1

        elif self.ir_step == 4:  # End code, if there is no end code, it is possible to receive a duplicate code and start from step=1
             if curtime >= 500 and curtime <= 650:
                 self.ir_step = 0

    def __check_cmd(self):
        byte4 = 0
        for i in range(32):
            x = i * 2
            t = self.buf64[x] + self.buf64[x+1]
            byte4 <<= 1
            if t >= 1800 and t <= 2800:
                byte4 += 1
        user_code_hi = (byte4 & 0xff000000) >> 24
        user_code_lo = (byte4 & 0x00ff0000) >> 16
        data_code = (byte4 & 0x0000ff00) >> 8
        data_code_r = byte4 & 0x000000ff
        self.cmd = data_code

    def scan(self):        
        # data received
        if self.recived_ok:
            self.__check_cmd()
            self.recived_ok = False
            
        # data has changed()
        if self.cmd != self.cmd_last or self.repeat != self.repeat_last or self.t_ok != self.t_ok_last:
            self.changed = True
        else:
            self.changed = False

        # renew
        self.cmd_last = self.cmd
        self.repeat_last = self.repeat
        self.t_ok_last = self.t_ok
        # Corresponding button character
        s = self.CODE.get(self.cmd)
        return self.changed, s, self.repeat, self.t_ok



class LcdApi:
    """Implements the API for talking with HD44780 compatible character LCDs.
    This class only knows what commands to send to the LCD, and not how to get
    them to the LCD.

    It is expected that a derived class will implement the hal_xxx functions.
    """

    # The following constant names were lifted from the avrlib lcd.h
    # header file, however, I changed the definitions from bit numbers
    # to bit masks.
    #
    # HD44780 LCD controller command set

    LCD_CLR = 0x01              # DB0: clear display
    LCD_HOME = 0x02             # DB1: return to home position

    LCD_ENTRY_MODE = 0x04       # DB2: set entry mode
    LCD_ENTRY_INC = 0x02        # --DB1: increment
    LCD_ENTRY_SHIFT = 0x01      # --DB0: shift

    LCD_ON_CTRL = 0x08          # DB3: turn lcd/cursor on
    LCD_ON_DISPLAY = 0x04       # --DB2: turn display on
    LCD_ON_CURSOR = 0x02        # --DB1: turn cursor on
    LCD_ON_BLINK = 0x01         # --DB0: blinking cursor

    LCD_MOVE = 0x10             # DB4: move cursor/display
    LCD_MOVE_DISP = 0x08        # --DB3: move display (0-> move cursor)
    LCD_MOVE_RIGHT = 0x04       # --DB2: move right (0-> left)

    LCD_FUNCTION = 0x20         # DB5: function set
    LCD_FUNCTION_8BIT = 0x10    # --DB4: set 8BIT mode (0->4BIT mode)
    LCD_FUNCTION_2LINES = 0x08  # --DB3: two lines (0->one line)
    LCD_FUNCTION_10DOTS = 0x04  # --DB2: 5x10 font (0->5x7 font)
    LCD_FUNCTION_RESET = 0x30   # See "Initializing by Instruction" section

    LCD_CGRAM = 0x40            # DB6: set CG RAM address
    LCD_DDRAM = 0x80            # DB7: set DD RAM address

    LCD_RS_CMD = 0
    LCD_RS_DATA = 1

    LCD_RW_WRITE = 0
    LCD_RW_READ = 1

    def __init__(self, num_lines, num_columns):
        self.num_lines = num_lines
        if self.num_lines > 4:
            self.num_lines = 4
        self.num_columns = num_columns
        if self.num_columns > 40:
            self.num_columns = 40
        self.cursor_x = 0
        self.cursor_y = 0
        self.backlight = True
        self.display_off()
        self.backlight_on()
        self.clear()
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)
        self.hide_cursor()
        self.display_on()

    def clear(self):
        """Clears the LCD display and moves the cursor to the top left
        corner.
        """
        self.hal_write_command(self.LCD_CLR)
        self.hal_write_command(self.LCD_HOME)
        self.cursor_x = 0
        self.cursor_y = 0

    def show_cursor(self):
        """Causes the cursor to be made visible."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def hide_cursor(self):
        """Causes the cursor to be hidden."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def blink_cursor_on(self):
        """Turns on the cursor, and makes it blink."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR | self.LCD_ON_BLINK)

    def blink_cursor_off(self):
        """Turns on the cursor, and makes it no blink (i.e. be solid)."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def display_on(self):
        """Turns on (i.e. unblanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def display_off(self):
        """Turns off (i.e. blanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL)

    def backlight_on(self):
        """Turns the backlight on.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = True
        self.hal_backlight_on()

    def backlight_off(self):
        """Turns the backlight off.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = False
        self.hal_backlight_off()

    def move_to(self, cursor_x, cursor_y):
        """Moves the cursor position to the indicated position. The cursor
        position is zero based (i.e. cursor_x == 0 indicates first column).
        """
        self.cursor_x = cursor_x
        self.cursor_y = cursor_y
        addr = cursor_x & 0x3f
        if cursor_y & 1:
            addr += 0x40    # Lines 1 & 3 add 0x40
        if cursor_y & 2:
            addr += 0x14    # Lines 2 & 3 add 0x14
        self.hal_write_command(self.LCD_DDRAM | addr)

    def putchar(self, char):
        """Writes the indicated character to the LCD at the current cursor
        position, and advances the cursor by one position.
        """
        if char != '\n':
            self.hal_write_data(ord(char))
            self.cursor_x += 1
        if self.cursor_x >= self.num_columns or char == '\n':
            self.cursor_x = 0
            self.cursor_y += 1
            if self.cursor_y >= self.num_lines:
                self.cursor_y = 0
            self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        """Write the indicated string to the LCD at the current cursor
        position and advances the cursor position appropriately.
        """
        for char in string:
            self.putchar(char)

    def custom_char(self, location, charmap):
        """Write a character to one of the 8 CGRAM locations, available
        as chr(0) through chr(7).
        """
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        time.sleep_us(40)
        for i in range(8):
            self.hal_write_data(charmap[i])
            time.sleep_us(40)
        self.move_to(self.cursor_x, self.cursor_y)

    def hal_backlight_on(self):
        """Allows the hal layer to turn the backlight on.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_backlight_off(self):
        """Allows the hal layer to turn the backlight off.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_write_command(self, cmd):
        """Write a command to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

    def hal_write_data(self, data):
        """Write data to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError
    

# The PCF8574 has a jumper selectable address: 0x20 - 0x27
DEFAULT_I2C_ADDR = 0x27

# Defines shifts(移位) or masks（掩码） for the various LCD line attached to the PCF8574

MASK_RS = 0x01
MASK_RW = 0x02
MASK_E = 0x04
SHIFT_BACKLIGHT = 3
SHIFT_DATA = 4


class I2cLcd(LcdApi):
    """Implements a HD44780 character LCD connected via PCF8574 on I2C."""

    def __init__(self, i2c, i2c_addr, num_lines, num_columns):
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.i2c.writeto(self.i2c_addr, bytearray([0]))
        sleep_ms(20)   # Allow LCD time to powerup
        # Send reset 3 times
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(5)    # need to delay at least 4.1 msec
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(1)
        # Put LCD into 4 bit mode
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        sleep_ms(1)
        LcdApi.__init__(self, num_lines, num_columns)
        cmd = self.LCD_FUNCTION
        if num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)

    def hal_write_init_nibble(self, nibble):
        """Writes an initialization nibble to the LCD.

        This particular function is only used during initialization.
        """
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))

    def hal_backlight_on(self):
        """Allows the hal layer to turn the backlight on."""
        self.i2c.writeto(self.i2c_addr, bytearray([1 << SHIFT_BACKLIGHT]))

    def hal_backlight_off(self):
        """Allows the hal layer to turn the backlight off."""
        self.i2c.writeto(self.i2c_addr, bytearray([0]))

    def hal_write_command(self, cmd):
        """Writes a command to the LCD.

        Data is latched(锁定) on the falling edge of E.
        """
        byte = ((self.backlight << SHIFT_BACKLIGHT) | (((cmd >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        byte = ((self.backlight << SHIFT_BACKLIGHT) | ((cmd & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        if cmd <= 3:
            # The home and clear commands require a worst case delay of 4.1 msec
            sleep_ms(5)

    def hal_write_data(self, data):
        """Write data to the LCD."""
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | (((data >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | ((data & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))

class LCD1602(I2cLcd):
    def __init__(self):
        self.DEFAULT_I2C_ADDR = 0x27
        self.i2c = I2C(0,sda=Pin(20),scl=Pin(21),freq=400000)
        
    def lcd(self):
        self.lcd = I2cLcd(self.i2c, self.DEFAULT_I2C_ADDR, 2, 16)