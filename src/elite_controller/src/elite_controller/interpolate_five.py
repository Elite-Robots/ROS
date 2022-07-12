import math


def get_five_fun(time, start_joint, end_joint, start_speed=0, end_speed=0, start_acc=0, end_acc=0):
    """五次项目，生成一段函数"""
    # print(time,start_joint,end_joint,start_speed,end_speed,start_acc,end_acc)
    tf = []
    for i in range(6):
        tf.append(math.pow(time, i))
    det = end_joint - start_joint
    a0 = start_joint
    a1 = start_speed
    a2 = start_acc/2
    a3 = (20*det - (8*end_speed+12*start_speed) *
          tf[1] - (3*start_acc-end_acc)*tf[2]) / (2*tf[3])
    a4 = (-30*det + (14*end_speed+16*start_speed) *
          tf[1] + (3*start_acc-2*end_acc)*tf[2]) / (2*tf[4])
    a5 = (12*det - (6*end_speed+6*start_speed) *
          tf[1] - (start_acc-end_acc)*tf[2]) / (2*tf[5])
    print(a0, a1, a2, a3, a4, a5)

    def joint(t):
        tf = []
        for i in range(6):
            tf.append(math.pow(t, i))
        d = a0+a1*tf[1]+a2*tf[2]+a3*tf[3]+a4*tf[4]+a5*tf[5]
        return d

    def speed(t):
        tf = []
        for i in range(6):
            tf.append(math.pow(t, i))
        d = a1+2*a2*tf[1]+3*a3*tf[2]+4*a4*tf[3]+5*a5*tf[4]
        return d

    def acceleration(t):
        tf = []
        for i in range(6):
            tf.append(math.pow(t, i))
        d = 2*a2+6*a3*tf[1]+12*a4*tf[2]+20*a5*tf[3]
        return d
    return joint, speed, acceleration


def get_path_fun(path, joint_index=0):
    """获取整个路径关于时间的函数"""
    sum_time = path[-1].time_from_start.to_sec()  # 总时间
    len_point = len(path)  # 总点数
    time_joint_fun = {}
    time_speed_fun = {}
    time_accelerate_fun = {}
    for i in range(len_point-1):
        time = path[i+1].time_from_start.to_sec() - \
            path[i].time_from_start.to_sec()
        start_joint = path[i].positions[joint_index]
        end_joint = path[i+1].positions[joint_index]
        start_speed = path[i].velocities[joint_index]
        end_speed = path[i+1].velocities[joint_index]
        start_acc = path[i].accelerations[joint_index]
        end_acc = path[i+1].accelerations[joint_index]
        f, s, a = get_five_fun(time, start_joint, end_joint,
                               start_speed, end_speed, start_acc, end_acc)
        time_joint_fun[path[i+1].time_from_start.to_sec()] = f
        time_speed_fun[path[i+1].time_from_start.to_sec()] = s
        time_accelerate_fun[path[i+1].time_from_start.to_sec()] = a

    def joint(jt):
        last_t = 0
        for t in time_joint_fun.keys():
            if jt <= t:
                return time_joint_fun[t](jt-last_t)
            last_t = t

    def speed(jt):
        last_t = 0
        for t in time_speed_fun.keys():
            if jt <= t:
                return time_speed_fun[t](jt-last_t)
            last_t = t

    def accelerate(jt):
        last_t = 0
        for t in time_accelerate_fun.keys():
            if jt <= t:
                return time_accelerate_fun[t](jt-last_t)
            last_t = t

    return sum_time, joint, speed, accelerate
