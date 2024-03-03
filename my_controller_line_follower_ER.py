from controller import Robot
from controller import Camera #import camera node
from controller import CameraRecognitionObject
from controller import DistanceSensor
time_step=32
max_speed = 6.28  # This is the maximum speed for the e-puck motors
robot = Robot()  
# Get handles for motors and ground sensors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
    
left_ir = Camera("left_linesensor")
left_ir.enable(time_step)
    
right_ir = Camera("right_linesensor")
right_ir.enable(time_step)

vis = Camera("camera")
vis.enable(time_step)
light_sensor=robot.getDevice("ls5")
light_sensor.enable(time_step)
ds_right = robot.getDevice("ps0")
ds_right.enable(time_step)
ds_right1 = robot.getDevice("ps1")
ds_right1.enable(time_step)
ds_right2 = robot.getDevice("ps2")
ds_right2.enable(time_step)

ds_left = robot.getDevice("ps7")
ds_left.enable(time_step)
ds_left1 = robot.getDevice("ps6")
ds_left1.enable(time_step)
ds_left2 = robot.getDevice("ps5")
ds_left2.enable(time_step)

left_speed= max_speed * 0.8
right_speed= max_speed * 0.8    
last_turn=0  
val=1 
val1=0
quit=0 
t=0
t1=0
t2=1
bound1=0
bound2=0
bound3=0
bound4=0
bound5=0
s=0
route=0
count=0
turn=0
add=0
# Main control loop
while robot.step(time_step) != -1:  # Step the simulation in 16ms increments
      # Read sensor values
      ls_val = light_sensor.getValue()
      #print('ls',ls_val)
      if ls_val==0:
         route=1
      if t==0:
         val_left = ds_left.getValue()
         val_left1 = ds_left1.getValue()
         val_left2 = ds_left2.getValue()
         val_right = ds_right.getValue()
         val_right1 = ds_right1.getValue()
         val_right2 = ds_right2.getValue()
      #print('vl',val_left)
      #print('v2',val_left1)
      #print('v3',val_left2)
      #print('v4',val_right)
      #print('v5',val_right1)
      #print('v6',val_right2) 
      if s==0:
         visD = vis.getImage() #get theimage detected by the camera
         vision = vis.imageGetGray(visD, vis.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
         print('Vis  = ',vision)
      if vision  ==  36.666666666666664:
         left_motor.setVelocity(0)
         right_motor.setVelocity(0)
         route=4
      if vision  ==  47.333333333333336:
         left_motor.setVelocity(0)
         right_motor.setVelocity(0)
      if vision  ==  53.0:
         s=0
         #s=1
         #last_turn=1
         #left_motor.setVelocity(0)
         #right_motor.setVelocity(0)
      if route==4:
         print(t2)
         if t2>0 and t2<15:
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(-right_speed)
            t2=t2+1
         if t2>14 and t2<80:
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            t2=t2+1
         if t2>79 and t2<95:
            left_motor.setVelocity(-left_speed)
            right_motor.setVelocity(right_speed)
            t2=t2+1
         if t2>94 and t2<220:
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            t2=t2+1
         if t2>219 and t2<235:
            left_motor.setVelocity(-left_speed)
            right_motor.setVelocity(right_speed)
            t2=t2+1
         if t2>234 and t2<280:
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            t2=t2+1
         if t2>279:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            t2=t2+1
           
      if route==1 or route==2:
         if vision  ==  56.0:
            bound1=26
            bound2=25
            bound3=15
            bound4=14
            bound5=0
         if vision==87.66666666666667:
            bound1=30
            bound2=29
            bound3=5
            bound4=4
            bound5=1
         if val1>bound4:
            #val1=0
            camDataR = right_ir.getImage() #get theimage detected by the camera
            grayR = right_ir.imageGetGray(camDataR, right_ir.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
            print('RGB_R  = ',grayR)
            camDataL = left_ir.getImage() #get theimage detected by the camera
            grayL = left_ir.imageGetGray(camDataL, right_ir.getWidth(), 5, 10)
            print('RGB_L  = ',grayL)
            if bound5==1:
               if t==1:
                  val_left=0
                  if t1<15:
                     if grayL<135 or grayR<135:
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(-right_speed)
                        t1=t1+1
                  if t1>14:
                     if grayL>135 and grayR<135:
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(-right_speed)
                     if grayL<135 and grayR>135:
                        left_motor.setVelocity(-left_speed)
                        right_motor.setVelocity(right_speed)                 
                     if grayL>135 and grayR>135:
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed) 
                     if grayL<135 and grayR<135:
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed)
                        print('G')
                        last_turn=0
                        
                        val=1
                        val1=0
                        t=0
                        t1=0
                        quit=quit+1
                        print(quit)
                             
               if val_left<1000 or val_left1<1000 or val_left2<1000:
                  if t==0:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(-right_speed)
                     if val_left2<1000:   
                        last_turn=1
                        val=1
                        val1=0
               if val_right<1000 or val_right1<1000 or val_right2<1000:
                  if t==0:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(right_speed)      
                     if val_right2<1000:
                        last_turn=2
                        val=1
                        val1=0
               if grayL<135 or grayR<135:
                  left_motor.setVelocity(left_speed)
                  right_motor.setVelocity(-right_speed)               
               if val_left==1000 and val_right==1000 and val_left1==1000 and val_right1==1000 and val_left2==1000 and val_right2==1000:    
                  if grayL>135 or grayR>135:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(-right_speed)
                     t=1
                  if grayL<135 and grayR<135:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(right_speed)
                     print('G')
                     last_turn=0
                     
                     val=1
                     val1=0
                     t=0
                     quit=quit+1
                     print(quit) 
            if bound5==0:
               if val_left<1000 or val_left1<1000 or val_left2<1000:
                  left_motor.setVelocity(left_speed)
                  right_motor.setVelocity(-right_speed)
                  if val_left2<1000:   
                     last_turn=1
                     val=1
                     val1=0
               if val_right<1000 or val_right1<1000 or val_right2<1000:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(right_speed)      
                  if val_right2<1000:
                     last_turn=2
                     val=1
                     val1=0
               if grayL<135 or grayR<135:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(right_speed)               
               if val_left==1000 and val_right==1000 and val_left1==1000 and val_right1==1000 and val_left2==1000 and val_right2==1000:    
                  if grayL<135 and grayR<135:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(right_speed)
                     print('G')
                     last_turn=0
                     val=1
                     val1=0
                     quit=quit+1
                     print(quit)           
         if val>bound2 and val1<bound3:
            val1=val1+1
            print('v',val1)
            if last_turn==1:
               left_motor.setVelocity(-left_speed)
               right_motor.setVelocity(right_speed)
            if last_turn==2:
               left_motor.setVelocity(left_speed)
               right_motor.setVelocity(-right_speed)   
         if val>0 and val<bound1:
            if last_turn==1 or last_turn==2:
               left_motor.setVelocity(left_speed)
               right_motor.setVelocity(right_speed)
               val=val+1
               print(val)        
         if last_turn==0:
            camDataR = right_ir.getImage() #get theimage detected by the camera
            grayR = right_ir.imageGetGray(camDataR, right_ir.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
            #print('RGB_R  = ',grayR)
            camDataL = left_ir.getImage() #get theimage detected by the camera
            grayL = left_ir.imageGetGray(camDataL, right_ir.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
            #print('RGB_L  = ',grayL)
         if last_turn==0:
            #print('La',last_turn)
            if val_left==1000 and val_right==1000 and val_left1==1000 and val_right1==1000 and val_left2==1000 and val_right2==1000:
               if route==1 and turn>5:
                  if grayL==139.66666666666666 and grayR<130.0:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(right_speed)
                     count=count+1
                     if count>10:
                        count=0
                        route=2
                        last_turn=2 
               if count<1:
                  if quit==3 or quit>3:
                     add=add+1
                     print('A',add)
                  if grayL<135 and grayR<135 and add<110:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(right_speed)
                  if grayL>135 and grayR<135 and add<110:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(-right_speed)
                  if grayL<135 and grayR>135 and add<110:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(right_speed)
                     turn=turn+1
                     #print(turn)                 
                  if grayL>135 and grayR>135 and add<110:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(-right_speed)
                  if add>109 and add<140:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(right_speed)
                  if add>139 and add<170:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(right_speed)   
               #add=add+1
                  if add>169:
                     left_motor.setVelocity(0)
                     right_motor.setVelocity(0)
            if add<110:
               if val_left<1000 or val_left1<1000 or val_left2<1000:
                  left_motor.setVelocity(left_speed)
                  right_motor.setVelocity(-right_speed)
                  if val_left<1000:
                     last_turn=1
               if val_right<1000 or val_right1<1000 or val_right2<1000:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(right_speed)      
                  if val_right<1000: 
                     last_turn=2 
      if route==0:
         if vision  ==  56.0:
            bound1=26
            bound2=25
            bound3=15
            bound4=14
            bound5=0
         if vision==87.66666666666667:
            bound1=35
            bound2=34
            bound3=15
            bound4=14
            bound5=1     
         if last_turn==0:
            camDataR = right_ir.getImage() #get theimage detected by the camera
            grayR = right_ir.imageGetGray(camDataR, right_ir.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
            #print('RGB_R  = ',grayR)
            camDataL = left_ir.getImage() #get theimage detected by the camera
            grayL = left_ir.imageGetGray(camDataL, right_ir.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
            #print('RGB_L  = ',grayL)
         if val1>bound4:
            #val1=0
            camDataR = right_ir.getImage() #get theimage detected by the camera
            grayR = right_ir.imageGetGray(camDataR, right_ir.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
            print('RGB_R  = ',grayR)
            camDataL = left_ir.getImage() #get theimage detected by the camera
            grayL = left_ir.imageGetGray(camDataL, right_ir.getWidth(), 5, 10)
            print('RGB_L  = ',grayL)
            if bound5==1:
               if t==1:
                  val_left=0
                  if t1<15:
                     if grayL<135 or grayR<135:
                        left_motor.setVelocity(-left_speed)
                        right_motor.setVelocity(right_speed)
                        t1=t1+1
                  if t1>14:
                     if grayL>135 and grayR<135:
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(-right_speed)
                     if grayL<135 and grayR>135:
                        left_motor.setVelocity(-left_speed)
                        right_motor.setVelocity(right_speed)                 
                     if grayL>135 and grayR>135:
                        left_motor.setVelocity(-left_speed)
                        right_motor.setVelocity(-right_speed) 
                     if grayL<135 and grayR<135:
                        left_motor.setVelocity(left_speed)
                        right_motor.setVelocity(right_speed)
                        print('G')
                        last_turn=0
                        val=1
                        val1=0
                        t=0
                        quit=quit+1
                        print(quit)  
               if val_left<1000 or val_left1<1000 or val_left2<1000:
                  if t==0:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(-right_speed)
                     if val_left2<1000:   
                        last_turn=1
                        val=1
                        val1=0
               if val_right<1000 or val_right1<1000 or val_right2<1000:
                  if t==0:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(right_speed)      
                     if val_right2<1000:
                        last_turn=2
                        val=1
                        val1=0
               if grayL<135 or grayR<135:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(right_speed)               
               if val_left==1000 and val_right==1000 and val_left1==1000 and val_right1==1000 and val_left2==1000 and val_right2==1000:    
                  if grayL>135 or grayR>135:
                     left_motor.setVelocity(-left_speed)
                     right_motor.setVelocity(-right_speed)
                     t=1
                  if grayL<135 and grayR<135:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(right_speed)
                     print('G')
                     last_turn=0
                     val=1
                     val1=0
                     quit=quit+1
                     print(quit)
            if bound5==0:
               if val_left<1000 or val_left1<1000 or val_left2<1000:
                  left_motor.setVelocity(left_speed)
                  right_motor.setVelocity(-right_speed)
                  if val_left2<1000:   
                     last_turn=1
                     val=1
                     val1=0
               if val_right<1000 or val_right1<1000 or val_right2<1000:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(right_speed)      
                  if val_right2<1000:
                     last_turn=2
                     val=1
                     val1=0
               if grayL<135 or grayR<135:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(right_speed)               
               if val_left==1000 and val_right==1000 and val_left1==1000 and val_right1==1000 and val_left2==1000 and val_right2==1000:    
                  if grayL<135 and grayR<135:
                     left_motor.setVelocity(left_speed)
                     right_motor.setVelocity(right_speed)
                     print('G')
                     last_turn=0
                     val=1
                     val1=0
                     quit=quit+1
                     print(quit)           
         if val>bound2 and val1<bound3:
            val1=val1+1
            print('v',val1)
            if last_turn==1:
               left_motor.setVelocity(-left_speed)
               right_motor.setVelocity(right_speed)
            if last_turn==2:
               left_motor.setVelocity(left_speed)
               right_motor.setVelocity(-right_speed)   
         if val>0 and val<bound1:
            if last_turn==1 or last_turn==2:
               left_motor.setVelocity(left_speed)
               right_motor.setVelocity(right_speed)
               val=val+1
               print(val)      
         if last_turn==0:
            #print('La',last_turn)
            if val_left==1000 and val_right==1000 and val_left1==1000 and val_right1==1000 and val_left2==1000 and val_right2==1000:
               if grayL<135 and grayR<135:
                  left_motor.setVelocity(left_speed)
                  right_motor.setVelocity(right_speed)
               if grayL>135 and grayR<135:
                  left_motor.setVelocity(left_speed)
                  right_motor.setVelocity(-right_speed)
               if grayL<135 and grayR>135:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(right_speed)                 
               if grayL>135 and grayR>135:
                  left_motor.setVelocity(-left_speed)
                  right_motor.setVelocity(-right_speed)      
            if val_left<1000 or val_left1<1000 or val_left2<1000:
               left_motor.setVelocity(left_speed)
               right_motor.setVelocity(-right_speed)
               if val_left<1000:
                  last_turn=1
            if val_right<1000 or val_right1<1000 or val_right2<1000:
               left_motor.setVelocity(-left_speed)
               right_motor.setVelocity(right_speed)      
               if val_right2<1000: 
                  last_turn=2               
      pass