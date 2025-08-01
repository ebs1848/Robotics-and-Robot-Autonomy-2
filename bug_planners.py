import numpy as np

def planner(robot,sensor,target_position):
    if robot.planner == 0:
        bug0(robot,sensor,target_position)
    elif robot.planner == 1:
        bug1(robot,sensor,target_position)
    elif robot.planner == 2:
        bug2(robot,sensor,target_position)
    elif robot.planner == 3:
        tangent_bug(robot,sensor,target_position)

####################################BUG0################################

def check_obstacle(position,target_position):
    vt=[target_position[0]-position.x,target_position[1]-position.y]
    vr=[np.cos(np.radians(position.theta)),np.sin(np.radians(position.theta))]
    direction = np.cross(vr,vt)
    if direction<=0:
        free_side=0
    else:
        free_side=1
    return free_side

def bug0(robot,bump_sensors,target_position):
    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in bump_sensors):
            robot.fsm = "wf"
        else:
            robot.fsm = "m2g"
    elif robot.fsm == "wf":
        if check_obstacle(robot.position,target_position):
            robot.fsm = "m2g"
        else:
            robot.fsm = "wf"

######################################BUG1###############################
def write_to_memory(robot,target_position):
    robot.memory.pos = np.append(robot.memory.pos,[robot.position.x,robot.position.y])
    d = np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))
    robot.memory.d = np.append(robot.memory.d,d)

def check_point_hit(position,point):
    dist = np.linalg.norm(np.array(point)-np.array([position.x,position.y]))
    return dist<1.25

def bug1(robot,bump_sensors,target_position):
    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in bump_sensors):
            robot.fsm = "wf"
            robot.touchpoint = [robot.position.x, robot.position.y]
        else:
            robot.fsm = "m2g"
    elif robot.fsm == "wf":
        if check_point_hit(robot.position,robot.touchpoint):
            robot.fsm = "m2lp"
            robot.leavepoint = np.array([robot.memory.pos[2*np.argmin(robot.memory.d)], robot.memory.pos[2*np.argmin(robot.memory.d)+1]])
        else:
            robot.fsm = "wf" 
    elif robot.fsm == "m2lp":
        if check_point_hit(robot.position,robot.leavepoint):
            if check_obstacle(robot.position,target_position):
                robot.fsm = "m2g"
            else:
                robot.fsm = "noSoln"
        else:
            robot.fsm = "m2lp"

#################################BUG2##################################
def calculate_mline(robot,target_position):
    if robot.position.x == target_position[0]:
        nf=1
        m=None
        c=robot.position.x
    else:
        nf=0
        m=(target_position[1]-robot.position.y)/(target_position[0]-robot.position.x)
        c=robot.position.y-m*robot.position.x
    robot.memory.bug2param=np.append(robot.memory.bug2param,[nf,m,c,None,0])

def check_mline_hit(robot):
    if robot.memory.bug2param[1]==1:
        return np.abs(robot.position.x-robot.memory.bug2param[3])<1.2
    else:
        m=robot.memory.bug2param[2]
        c=robot.memory.bug2param[3]
        dist=np.abs(robot.position.y-m*robot.position.x-c)/(np.sqrt(1+m*m))
        return dist<1.2
    
def bug2(robot,bump_sensors,target_position):
    if robot.memory.bug2param[0]==1:
        robot.memory.bug2param[0]=0
        calculate_mline(robot,target_position)

    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in bump_sensors) and not robot.memory.bug2param[5]:
            robot.fsm = "wf"
            dmin=np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))
            robot.memory.bug2param[4]=dmin
        else:
            robot.fsm = "m2g"
            robot.memory.bug2param[5]=0
    elif robot.fsm == "wf":
        d=np.linalg.norm(target_position-np.array([robot.position.x,robot.position.y]))
        if check_mline_hit(robot) and d<robot.memory.bug2param[4] and check_obstacle(robot.position,target_position):
            robot.fsm = "m2g"
            robot.memory.bug2param[5]=1
        else:
            robot.fsm = "wf"

####################################BUG0################################
def tangent_bug(robot,lidar_sensor,target_position):
    # The code below is the same with bug 0. Make the necessary changes and additions for tangent bug
    if robot.fsm == "m2g":
        if any(sensor.activated for sensor in lidar_sensor):
            robot.fsm = "wf"
        else:
            robot.fsm = "m2g"
    elif robot.fsm == "wf":
        if check_obstacle(robot.position,target_position):
            robot.fsm = "m2g"
        else:
            robot.fsm = "wf"