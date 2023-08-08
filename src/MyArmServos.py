#时间 数字库
import time,math
#单片机 定时器

#舵机的驱动

#舵机0-180 （需要时间 时间长）

#舵机动   （时间短）

#获取当前舵机角度
from pyb import Pin, Timer
class ArmServos:
    def __init__(self):#初始化
        tim3= Timer(3, freq=50) # Frequency in Hz
        tim4= Timer(4, freq=50) # Frequency in Hz
        #pwm信号
        self.__Servo1 = tim4.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)#D12
        self.__Servo2 = tim4.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width_percent=75)#13
        self.__Servo3 = tim4.channel(3, Timer.PWM, pin=Pin("P9"), pulse_width_percent=50)
        self.__Servo4 = tim4.channel(4, Timer.PWM, pin=Pin("P10"), pulse_width_percent=50)
        self.__Servo5 = tim3.channel(3, Timer.PWM, pin=Pin("C8"), pulse_width_percent=50)
        self.__Servo6 = tim3.channel(4, Timer.PWM, pin=Pin("C9"), pulse_width_percent=50)

        self.__Servo1.pulse_width_percent(0)#转盘  4右7中10左
        self.__Servo2.pulse_width_percent(0)#舵机1 越大越往前倾
        self.__Servo3.pulse_width_percent(0)#舵机2 越大越往前倾
        self.__Servo4.pulse_width_percent(0)#舵机3 越大越往后倒
        self.__Servo5.pulse_width_percent(0)#舵机2 越大越往前倾
        self.__Servo6.pulse_width_percent(0)#舵机3 越大越往后倒

        self.__Servo1.pulse_width(2800)#4800 960
        self.__Servo2.pulse_width(2800)#1900 4800
        self.__Servo3.pulse_width(2800)#4800 960
        self.__Servo4.pulse_width(2800)#4800 960
        self.__Servo5.pulse_width(2800)#4800 960
        self.__Servo6.pulse_width(2800)#4800 960

        self.angleNow=[90,90,90,90,90,90]#记录每一个关节 当前角度
    #0-180（0-180距离）
    def __run(self,who,begin,end):
        begin=int(begin*21.4+960)
        end=int(end*21.4+960)
        self.angleNow[who-1]=end
        if (who==1):
            self.__duoji_control(self.__Servo1,begin,end)
        if (who==2):
            self.__duoji_control(self.__Servo2,begin,end)
        if (who==3):
            self.__duoji_control(self.__Servo3,begin,end)
        if (who==4):
            self.__duoji_control(self.__Servo4,begin,end)
        if (who==5):
            self.__duoji_control(self.__Servo5,begin,end)
        if (who==6):
            self.__duoji_control(self.__Servo6,begin,end)
    def RunNoDealy(self,who,begin,end):
        if(begin<0):
            begin=0
        if(begin>180):
            begin=180
        if(end<0):
            end=0
        if(end>180):
            end=180
        self.angleNow[who-1]=end
        begin=int(begin*21.4+960)
        end=int(end*21.4+960)
        if (who==1):
            self.__Servo1.pulse_width(int(end))
        if (who==2):
            self.__Servo2.pulse_width(int(end))
        if (who==3):
            self.__Servo3.pulse_width(int(end))
        if (who==4):
            self.__Servo4.pulse_width(int(end))
        if (who==5):
            self.__Servo5.pulse_width(int(end))
        if (who==6):
            self.__Servo6.pulse_width(int(end))
    def ArmInit(self,angleInit=[90,90,90,90,90,90]):
        for i in range(1,7):
            self.__run(i,90,angleInit[i-1])
        self.angleNow=angleInit
    def ArmControl(self,angleEnd=[90,90,90,90,90,90]):
        for i in range(1,7):
            self.__run(i,self.angleNow[i-1],angleEnd[i-1])
        self.angleNow=angleEnd
    def GetNowAngle(self):
        print(self.angleNow)
        return self.angleNow
    def __duoji_control(self,Servox,angle_begin,angle_end):
        angle_error= int((angle_end - angle_begin))
        angle_temp=angle_begin
        if(angle_error<0):
            for i in range(0,angle_error,-1):
                angle_temp=angle_temp - 1
                Servox.pulse_width(int(angle_temp))
                time.sleep_us(int(-angle_error/2))
        if(angle_error>0):
            for i in range(0,angle_error,1):
                angle_temp=angle_temp + 1
                Servox.pulse_width(int(angle_temp))
                time.sleep_us(int(angle_error/2))
