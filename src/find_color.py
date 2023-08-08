import sensor, image, time,lcd,pyb
from MyArmServos import ArmServos
arm=ArmServos()
from pid import PID
roi=[0,0,160,120]
obj_threshold  =[ (27, 68, -58, -19, 4, 81),#黄色
                  (17, 61, 13, 67, -9, 71),#红色
               (20, 59, 13, 87, -102, 88)]#蓝色
color_flag=0 #定义当前抓取颜色
angle=[80,90,90,90,90,90]
arm.ArmInit(angle)
key = pyb.Pin('C13', pyb.Pin.IN)
#pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.27, i=0.5, d=0,imax=5)#在线调试使用这个PID
#tilt_pid = PID(p=0.5, i=0.2, imax=90)#在线调试使用这个PID

pan_pid = PID(p=0.03, i=0.4,imax=100)#在线调试使用这个PID
tilt_pid = PID(p=0.03, i=0, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
sensor.set_vflip(False)
sensor.set_hmirror(True)
lcd.init(2)
clock = time.clock() # Tracks FPS.
angle=[90,90,90,90,100,50]
#angle[1]=60
#angle[2]=30
#angle[3]=135
angle[1]=50
angle[2]=50
angle[3]=180
arm.ArmControl(angle)
k=0
#angle=[90,90,0,20,90,90]
#angle[1]=140
#arm.ArmControl(angle)
#angle=[90,140,0,20,90,90]
flag=0
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob
def arm_panControl(pan_input):
    angle=[90,90,90,90,90,90]
    angle[0]=angle[0]+pan_input*0.5
    #if angle[0]>120:
        #angle[0]=120
    #if angle[0]<50:
        #angle[0]=50
    arm.RunNoDealy(1,arm.angleNow[0],angle[0])
    arm.GetNowAngle()
def arm_tiltControl(tilt_input):
    #angle=[90,55,30,180,100,90]
    angle=arm.GetNowAngle()
    angle[1]=angle[1]+(tilt_input)*1.7
    angle[2]=angle[2]-tilt_input*0.03
    angle[3]=angle[3]-tilt_input*0.7
    if(angle[1]>130):
        angle[1]=130
    arm.RunNoDealy(2,arm.angleNow[1],angle[1])
    arm.RunNoDealy(3,arm.angleNow[2],angle[2])
    arm.RunNoDealy(4,arm.angleNow[3],angle[3])
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    blobs = img.find_blobs([obj_threshold[color_flag]],roi=roi)
    img.draw_rectangle(roi,color=(0,255,0) )#
    if not blobs:
        color_flag=color_flag+1
        if(color_flag>2):
            color_flag=0
        #angle=arm.GetNowAngle()
        #angle[3]=angle[3]-1
        #arm.RunNoDealy(4,arm.angleNow[3],angle[3])
    if blobs:
        max_blob = find_max(blobs)
        pan_error= max_blob.cx()-img.width()/2
        tilt_error= max_blob.cy()-img.height()/2

        img.draw_rectangle(roi,color=(0,255,0) )# rect
        img.draw_rectangle(max_blob.rect()) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy

        pan_output=pan_pid.get_pid(pan_error,1)
        tilt_output=tilt_pid.get_pid(tilt_error,1)

        print("pan_output",pan_output)
        #if(tilt_output<20 and tilt_output >-25):
        arm_panControl(pan_output)#//-21--43
        arm_tiltControl(-tilt_output)
        print("tilt_error: ",tilt_output)#//60--+-30
        if(max_blob.area()>1440):
            time.sleep_ms(300)
            print("AREA",max_blob.area())
            arm.RunNoDealy(6,50,85)
            time.sleep_ms(1500)
            while True :
                img = sensor.snapshot()
                angle=[90,90,90,90,100,85]
                angle[1]=50
                angle[2]=50
                angle[3]=180
                arm.ArmControl(angle)
                angle=[90,90,90,90,100,85]
                angle[0]=180
                angle[1]=100
                angle[2]=30
                angle[3]=135
                angle[5]=50
                arm.ArmControl(angle)
                time.sleep_ms(100)
                angle=[90,90,90,90,100,50]
                angle[0]=90
                angle[1]=50
                angle[2]=50
                angle[3]=180
                arm.ArmControl(angle)
                color_flag=color_flag+1
                if(color_flag>2):
                    color_flag=0
                flag=1
                break
    while flag==1:
        img = sensor.snapshot()
        lcd.display(img)
        time.sleep_ms(2000)
        flag=0
    lcd.display(img)
