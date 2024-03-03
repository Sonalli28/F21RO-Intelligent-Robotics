from controller import Robot
from controller import Camera #import camera node
from controller import CameraRecognitionObject
from controller import DistanceSensor
from controller import LightSensor
time_step=32
max_speed = 6.28  # This is the maximum speed for the e-puck motors
robot = Robot()  
# Get handles for motors and ground sensors
LEFT_WHEEL = robot.getDevice("left wheel motor")
RIGHT_WHEEL = robot.getDevice("right wheel motor")
LEFT_WHEEL.setPosition(float('inf'))
RIGHT_WHEEL.setPosition(float('inf'))
LEFT_WHEEL.setVelocity(0.0)
RIGHT_WHEEL.setVelocity(0.0)
    
LEFT_LINESENSOR = Camera("LEFT_LINESENSOR")
LEFT_LINESENSOR.enable(time_step)
    
RIGHT_LINESENSOR = Camera("RIGHT_LINESENSOR")
RIGHT_LINESENSOR.enable(time_step)

GOAL_SENSOR = Camera("camera")
GOAL_SENSOR.enable(time_step)

LIGHT_DETECTOR=robot.getDevice("ls5")
LIGHT_DETECTOR.enable(time_step)

DS_RIGHT = robot.getDevice("ps0")
DS_RIGHT.enable(time_step)
DS_RIGHT1 = robot.getDevice("ps1")
DS_RIGHT1.enable(time_step)
DS_RIGHT2 = robot.getDevice("ps2")
DS_RIGHT2.enable(time_step)

DS_LEFT = robot.getDevice("ps7")
DS_LEFT.enable(time_step)
DS_LEFT1 = robot.getDevice("ps6")
DS_LEFT1.enable(time_step)
DS_LEFT2 = robot.getDevice("ps5")
DS_LEFT2.enable(time_step)

LEFT_VELOCITY= max_speed * 0.8
RIGHT_VELOCITY= max_speed * 0.8    
PREVIOUS_STATE=0  
N=1 
N1=0
Q=0 
PTR=0
PTR1=0 
PTR2=0
PTR3=0
PTR4=0
PTR5=0
PTR6=0
PTR7=0
PTR8=0
PTR9=0
PTR10=0
PTR11=0
PTR12=0
LS=0
GS=0
GD=0
LR = 0
LR1 = 0
LR2 = 0
LR4 = 0
LR5 = 0
LR3 = 0

def BBR():
      global PREVIOUS_STATE, N, N1, Q, PTR, PTR1, PTR2, PTR3,PTR4, PTR5, PTR6, PTR7, PTR8, PTR9, PTR10, PTR11, PTR12, LS, GD, GS, LR, LR1, LR2, LR4, LR5, LR3
      # Read sensor values
      LS = LIGHT_DETECTOR.getValue()
      #print('ls',LS)
      if LS==0:
         PTR8=1
      if PTR==0:
         LR = DS_LEFT.getValue()
         LR1 = DS_LEFT1.getValue()
         LR2 = DS_LEFT2.getValue()
         LR4 = DS_RIGHT.getValue()
         LR5 = DS_RIGHT1.getValue()
         LR3 = DS_RIGHT2.getValue()
      if PTR7==0 and N1==0:
         GD = GOAL_SENSOR.getImage() #get theimage detected by the camera
         GS = GOAL_SENSOR.imageGetGray(GD, GOAL_SENSOR.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
         print('GOAL_SENSOR  = ',GS)
      if GS  ==  53.0:
         PTR7=0
      if GS  ==  56.0 or GS == 39.666666666666664:
         if PTR8==2:
            PTR2=120
            PTR3=119
            PTR4=80
            PTR5=79
            PTR6=0
            PTR12=248
            print('Box')
         if PTR8==0:
            PTR2=150
            PTR3=149
            PTR4=10
            PTR5=9
            PTR6=0
            PTR12=248
            print('Box')
      if GS==87.66666666666667:
         PTR2=35
         PTR3=34
         PTR4=25
         PTR5=24
         PTR6=1
         PTR12=50
         print('Cylinder')     
      if PREVIOUS_STATE==0 and PTR11==0:
         camDataR = RIGHT_LINESENSOR.getImage() #get theimage detected by the camera
         grayR = RIGHT_LINESENSOR.imageGetGray(camDataR, RIGHT_LINESENSOR.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
         #print('RGB_R  = ',grayR)
         camDataL = LEFT_LINESENSOR.getImage() #get theimage detected by the camera
         grayL = LEFT_LINESENSOR.imageGetGray(camDataL, RIGHT_LINESENSOR.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
         #print('RGB_L  = ',grayL)
      if N1>PTR5:
         #N1=0
         camDataR = RIGHT_LINESENSOR.getImage() #get theimage detected by the camera
         grayR = RIGHT_LINESENSOR.imageGetGray(camDataR, RIGHT_LINESENSOR.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
         #print('RGB_R  = ',grayR)
         camDataL = LEFT_LINESENSOR.getImage() #get theimage detected by the camera
         grayL = LEFT_LINESENSOR.imageGetGray(camDataL, RIGHT_LINESENSOR.getWidth(), 5, 10)
         #print('RGB_L  = ',grayL)
         if PTR6==1:
            if PTR8==2:
               if N1<PTR12:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  N1=N1+1
                  print('v',N1)
               if PTR1<5:
                   if grayL<135 or grayR<135:
                      LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                      RIGHT_WHEEL.setVelocity(-RIGHT_VELOCITY)
                      PTR1=PTR1+1
               if PTR1>4:
                  if grayL>135 and grayR<135:
                     LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                     RIGHT_WHEEL.setVelocity(-RIGHT_VELOCITY)
                  if grayL<135 and grayR>135:
                     LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                     RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)                 
                  if grayL>135 and grayR>135:
                     LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                     RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY) 
                  if grayL<135 and grayR<135:
                     LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                     RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                     print('G')
                     PREVIOUS_STATE=0
                     N=1
                     N1=0
                     PTR=0
                     PTR1=0
                     Q=Q+1
                     print(Q)
                             
         if PTR6==0:
            if PTR8==0:
               if PTR1<9:
                   if grayL<135 or grayR<135:
                      LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                      RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                      PTR1=PTR1+1
                      print(PTR1)
               if PTR1>8 and PTR1<15:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>14 and PTR1<25:
                   
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>24 and PTR1<35:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>34 and PTR1<38:
                   
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>37 and PTR1<45:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>44 and PTR1<48:
                   
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>47 and PTR1<55:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)              
               if PTR1>54 and PTR1<58:
                   
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>57 and PTR1<65:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)  
               if PTR1>64:
                  
                  GD = GOAL_SENSOR.getImage() #get theimage detected by the camera
                  GS = GOAL_SENSOR.imageGetGray(GD, GOAL_SENSOR.getWidth(), 5, 10) #get the gray level present in the image detected by the camera
                  print('GOAL_SENSOR  = ',GS)
                  if GS==49.333333333333336:
                     LEFT_WHEEL.setVelocity(0)
                     RIGHT_WHEEL.setVelocity(0)
            if PTR8==2:
               if N1<PTR12:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  N1=N1+1
                  print('v',N1)
      
               if PTR1<4:
                   if grayL<135 or grayR<135:
                      LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                      RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                      PTR1=PTR1+1
                      print(PTR1)
               if PTR1>3 and PTR1<55:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)    
               if PTR1>54 and PTR1<85:
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>84 and PTR1<100:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR1=PTR1+1
                  print(PTR1)
               if PTR1>99:   
                  print('G')
                  PTR7=1
                  LEFT_WHEEL.setVelocity(0)
                  RIGHT_WHEEL.setVelocity(0)
                  Q=Q+1
                  print(Q)         
      if N>PTR3 and N1<PTR4:
         N1=N1+1
         print('v',N1)
         if PREVIOUS_STATE==1:
            LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
            RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
         if PREVIOUS_STATE==2:
            LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
            RIGHT_WHEEL.setVelocity(-RIGHT_VELOCITY)   
      if N>0 and N<PTR2:
         if PREVIOUS_STATE==1 or PREVIOUS_STATE==2:
            LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
            RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
            N=N+1
            print(N)      
      if PREVIOUS_STATE==0:
         if LR==1000 and LR4==1000 and LR1==1000 and LR5==1000 and LR2==1000 and LR3==1000:
            if PTR8==1 and PTR10>5:
               if grayL==139.66666666666666 and grayR<130.0:
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR9=PTR9+1
                  if PTR9>10:
                     PTR9=0
                     PTR8=2
            if PTR9<1:
               if grayL<135 and grayR<135:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
               if grayL>135 and grayR<135:
                  LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(-RIGHT_VELOCITY)
               if grayL<135 and grayR>135:
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)
                  PTR10=PTR10+1
                  #print(PTR10)                 
               if grayL>135 and grayR>135:
                  LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
                  RIGHT_WHEEL.setVelocity(-RIGHT_VELOCITY)      
         if LR4<1000 or LR5<1000 or LR3<1000:
            LEFT_WHEEL.setVelocity(-LEFT_VELOCITY)
            RIGHT_WHEEL.setVelocity(RIGHT_VELOCITY)      
            PTR11=1 
            PTR9=2
            if PTR8==0:
               PREVIOUS_STATE=1
               PTR11=0
            print('h',PTR11)
            if LR3<1000: 
               PREVIOUS_STATE=2
               PTR11=0
               PTR9=0
               print(PREVIOUS_STATE)
         if LR<1000 or LR1<1000 or LR2<1000:
            LEFT_WHEEL.setVelocity(LEFT_VELOCITY)
            RIGHT_WHEEL.setVelocity(-RIGHT_VELOCITY)
            PTR11=1
            if PTR8==0:
               PREVIOUS_STATE=1
               PTR11=0
            #PTR9=2
            print('h',PTR11)
            if LR2<1000:
               PREVIOUS_STATE=1
               PTR11=0
               #PTR9=0
               print(PREVIOUS_STATE) 

# Main control loop
while robot.step(time_step) != -1:  # Step the simulation in 16ms increments
      BBR()             
      pass