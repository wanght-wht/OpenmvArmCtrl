import sensor, image, time,math
import MyArmMath
from MyArmServos import ArmServos
arm=ArmServos()
from pid import PID
roi=[20,35,280,200]
obj_threshold  = (0,49,24, 83, 14, 55)
angle=[80,90,90,90,90,90]
arm.ArmInit(angle)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
angel1,angel2,angel3,angel4=MyArmMath.test_ok(-9,9,0)
angle=[angel1,angel2,angel3,angel4,90,90]
arm.ArmControl(angle)
print(angel3)
clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    img.draw_rectangle(roi,color=(0,255,0) )# rect
    img.draw_line((167,33,167,234),(0,0,255))


