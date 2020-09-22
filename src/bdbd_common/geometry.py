# Common geometry methods

import tf
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, PointStamped
from bdbd_common.utils import fstr

D_TO_R = math.pi / 180. # degrees to radians
TWOPI = 2. * math.pi

def poseDistance(pose1, pose2):
    # calculate the distance between two PoseStamped types in the same frame
    if pose1.header.frame_id != pose2.header.frame_id:
        raise RuntimeError('poses must be in the same frame')
    p1 = pose1.pose.position
    p2 = pose2.pose.position
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def poseTheta(pose1, pose2):
    # calculate the rotation on z axis between two PoseStamped types
    if pose1.header.frame_id != pose2.header.frame_id:
        raise RuntimeError('poses must be in the same frame')
    q1 = q_to_array(pose1.pose.orientation)
    q2 = q_to_array(pose2.pose.orientation)
    a1 = tf.transformations.euler_from_quaternion(q1)
    a2 = tf.transformations.euler_from_quaternion(q2)
    return (a2[2] - a1[2])

def q_to_array(orientation):
    # return 4-element array from geometry_msgs/Quaternion.msg
    return [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ]

def array_to_q(array):
    #return geometry_msgs/Quaternion.msg from 4-element list
    q = Quaternion()
    q.x = array[0]
    q.y = array[1]
    q.z = array[2]
    q.w = array[3]
    return q

def zeroPose(frame):
    # a zero PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time(0)
    pose.pose.orientation.w = 1.0
    return pose

def zeroPoint(frame):
    # a zero PointStamped
    point = PointStamped()
    point.header.frame_id = frame
    point.header.stamp = rospy.Time(0)
    return point

def rotationCenter(frame, vx, vy, omega):
    # center of rotation in frame. See RKJ notebook 2020-07-10
    r = vx / omega
    a = vy / omega
    #print(r, a)

    center = zeroPoint(frame)
    center.point.x = -a
    center.point.y = r
    return center

def shortestPath( x, y, phi, rho):
    '''
        shortest path from (0, 0) to a point at (x, y), with ending orientation phi from current
        orientation. Travel in either straight lines, or on circles of radius rho.

        See RKJ notebook circa 2020-08-16
    '''

    # motion circles
    # sc : start circle
    sc_ccw = [0.0, rho]
    sc_cw = [0.0, -rho]

    # fc: end_circle
    fc_ccw = (x - rho * math.sin(phi), y + rho * math.cos(phi))
    fc_cw = (x + rho * math.sin(phi), y - rho * math.cos(phi))

    A = [0., 0.] # starting point
    B = [x, y]   # ending point
    solutions = []
    # direction: 1 == ccw, -1 == cw
    for start_dir in (1, -1):
        for end_dir in (1, -1):
            C = sc_ccw if start_dir == 1 else sc_cw  # start motion circle center
            D = fc_ccw if end_dir == 1 else fc_cw    # end motion circle center
            a = D[0] - C[0]
            b = D[1] - C[1]
            theta = math.atan2(b, a)
            tsq = a**2 + b**2
            if start_dir != end_dir and tsq - 4. * rho **2 < 0.:
                #print('dir: {} {} invalid'.format(start_dir, end_dir))
                pass
            else:
                if start_dir == end_dir:
                    ssq = tsq
                    beta = theta if start_dir == 1  else -theta
                else:
                    ssq = tsq - 4. * rho**2
                    psi = math.acos(2. * rho / math.sqrt(tsq))
                    alpha = psi - theta if start_dir == 1 else psi + theta
                    beta = math.pi/ 2. - alpha
                s = math.sqrt(ssq)
                beta = beta % TWOPI

                E = [rho * math.sin(beta), rho * (1. - math.cos(beta))] # transition from start circle to line
                if start_dir == -1:
                    E[1] = -E[1]
                # F is transition from line to final circle See RKJ 2020-09-22
                if start_dir == 1:
                    if end_dir == 1:
                        F = [D[0] + rho * math.sin(beta), D[1] - rho * math.cos(beta)]
                    else:
                        F = [D[0] - rho * math.sin(beta), D[1] + rho * math.cos(beta)]
                else:
                    if end_dir == 1:
                        F = [D[0] - rho * math.sin(beta), D[1] - rho * math.cos(beta)]
                    else:
                        F = [D[0] + rho * math.sin(beta), D[1] + rho * math.cos(beta)]

                # RKJ notebook 2020-08-26
                if start_dir == 1 and end_dir == 1:
                    gamma = phi - beta
                elif start_dir == 1 and end_dir == -1:
                    gamma = beta - phi
                elif start_dir == -1 and end_dir == 1:
                    gamma = beta + phi
                else:
                    gamma = -beta - phi

                gamma = gamma % TWOPI
                length = s + rho * beta + rho * gamma
                solution = {'dir': (start_dir, end_dir), 'length': length, 'E': E, 'F': F, 'beta': beta, 'gamma': gamma}
                solutions.append(solution)

    # determine the best solution
    solution = solutions[0]
    print(fstr(solution))
    for i in range(1, len(solutions)):
        print(fstr(solutions[i]))
        if solutions[i]['length'] < solution['length']:
            solution = solutions[i]

    # return a path plan
    first_arc = {
        'start': (0.0, 0.0, 0.0),
        'end': (solution['E'][0], solution['E'][1], solution['beta']),
        'center': sc_ccw if solution['dir'][0] == 1 else sc_cw,
        'radius': rho,
        'angle': solution['beta'] * solution['dir'][0]
    }
    second_segment = {
        'start': first_arc['end'],
        'end': (solution['F'][0], solution['F'][1], solution['beta'])
    }
    third_arc = {
        'start': second_segment['end'],
        'end': (x, y, phi),
        'center': fc_ccw if solution['dir'][1] == 1 else fc_cw,
        'radius': rho,
        'angle': solution['gamma'] *  solution['dir'][1]
    }
    motion_plan = [first_arc, second_segment, third_arc]
    return motion_plan

def ccwPath(phi, x, y):
    '''
        determine a path from (0, 0) to a point at (x, y), with ending orientation phi from current
        orientation. Path consists of two circles with the same radius, intersecting at a tangent.
        See RKJ notebook circa 2020-09-12. This solution is just for initial CCW rotation. Negate
        phi, Y, ey,  beta for CW rotation.
    '''
    A = 1.0 - math.cos(phi)
    B = y * (1. + math.cos(phi)) - x * math.sin(phi)
    C = - (x**2 + y**2) / 2.

    # alternate form of quadratic equation, allows zero A
    try:
        rho = 2. * C / (-B - math.sqrt(B**2 - 4. * A * C))
    except ZeroDivisionError:
        return ccwPath(phi + 1.e-6, x, y)

    #   Calculate diagram values

    # x distance from target point to second circle center
    g = rho * math.sin(phi)
    # x distance between circle centers
    a = x + g
    # y distance between circle centers.
    b = rho * (1. + math.cos(phi)) - y
    # arc angle on first circle. Also orientation of robot at intersection point.
    beta = math.atan2(a , b)
    # intersection coordinates of circle tangents
    e = [rho * math.sin(beta), rho * (1. - math.cos(beta))]
    # arc angle for second circle. If negative, solution is invalid.
    gamma = beta - phi
    returns = {}
    for v in ['rho', 'beta', 'e', 'gamma', 'a', 'b']:
        returns[v] = eval(v)
    print(fstr(returns))
    return returns

def nearPath(x, y, phi):
    # see ccw path
    path = ccwPath(phi, x, y)
    if path['gamma'] < 0.0 or path['beta'] < 0.0:
        # try a clockwise rotation
        path = ccwPath(-phi, x, -y)
        if path['gamma'] < 0.0 or path['beta'] < 0.0:
            # solution requires two circles of same rotation, which we do not support
            return None
        path['beta'] *= -1
        path['e'][1] *= -1
    # calculate the motion plan
    if path:
        first_arc = {
            'start': (0.0, 0.0, 0.0),
            'end': (path['e'][0], path['e'][1], path['beta']),
            'center': (0.0, math.copysign(path['rho'], path['beta'])),
            'radius': path['rho'],
            'angle': path['beta']
        }
        second_arc = {
            'start': first_arc['end'],
            'end': (x, y, phi),
            'center': (path['a'], path['rho'] - path['b']),
            'radius': path['rho'],
            'angle': -math.copysign(path['gamma'], path['beta'])
        }
        motion_plan = [first_arc, second_arc]
    else:
        motion_plan = None

    return motion_plan

def b_to_w(base_pose, dx):
    # given the base pose, return the wheel pose
    theta = base_pose[2]
    return [
        base_pose[0] + dx * math.cos(theta),
        base_pose[1] + dx * math.sin(theta),
        theta
    ]

def w_to_b(wheel_pose, dx):
    theta = wheel_pose[2]
    return [
        wheel_pose[0] - dx * math.cos(theta),
        wheel_pose[1] - dx * math.sin(theta),
        theta
    ]

def lrEstimate(path, lr_model, start_twist, dt=0.025, left0 = 0.0, right0 = 0.0):
    # estimate robot left, right values to achieve a certain path, stopping with zero twist
    (pxl, pxr, fx) = lr_model[0]
    (pol, por, fo) = lr_model[2]
    beta = path['beta']
    rho = path['rho']
    gamma = path['gamma']
    length1 = abs(beta * rho)
    length2 = abs(gamma * rho)
    print(fstr({'length1': length1, 'length2': length2}))

    start_v = start_twist[0]
    start_o = start_twist[2]

    vmax = (pxl + pxr) / fx # x speed when left, right = 1.
    omegamax = (por - pol) / fo # omega when left, right = 1.
    alpha = omegamax / vmax

    # We assume that the actual velocity once the power is applied is related to the required omega. See
    # RKJ notebook 2020-09-14

    # vhat, omegahat are max values assuming the other is zero
    vhat = abs(start_v) + start_o / alpha
    omegahat = alpha * vhat

    # velocity will vary from current velocity, to end velocity. We have to cover length1 + length2 distance.
    # create an initial guess of left and right motors

    v_1 = vhat / (1. + vhat/(rho * omegahat))
    o_1 = math.copysign(v_1 / rho, beta)
    o_2 = -o_1
    print(fstr({'v_1': v_1, 'o_1': o_1, 'o_2:': o_2}))

    # velocity will vary linearly with time, so the required time can be estimated from the velocity.
    net_time = 2. * (length1 + length2) / v_1

    t = 0.0
    lrs = [{'t': 0.0, 'left':left0, 'right': right0}]
    s = 0.0 # curving path length
    # v, omega to left, right parameters see RKJ 2020-09-14
    a = pxl / fx
    b = pxr / fx
    c = pol / fo
    d = por / fo
    denom_left = b * d - a * c
    denom_right = b * c - a * d
    omega = start_o
    while t < net_time:
        t += dt
        # linear change of v, omega with time
        v = (1. - t / net_time) * v_1
        s += dt * v
        if s < length1:
            omega = (1. - t / net_time) * o_1
        else:
            omega = (1. - t / net_time) * o_2

        # apply the motor model
        left = (v * d - omega * b) / denom_left
        right = (v * c - omega * a) / denom_right
        lrs.append({'t': t, 'left': left, 'right': right})
        print(fstr({'t': t, 'left': left, 'right': right, 's': s, 'v': v, 'omega': omega}))

    return lrs

def default_lr_model():
    # vx model
    pxl = 1.258
    pxr = 1.378
    fx = 7.929

    # vy model
    pyl = -.677
    pyr = .657
    fy = 5.650

    # omega model
    pol = -7.659 # note this is negative of previous model, for consistency.
    por = 7.624
    fo = 8.464
    return ((pxl, pxr, fx), (pyl, pyr, fy), (pol, por, fo))

def dynamic_motion(lrs, start_pose=None, start_twist=None, lr_model=None):
    # apply the dynamic model in lr_model to the (left, right) values in lrs
    # model is in what we will call here 'base' frame of robot
    if lr_model is None:
        lr_model = default_lr_model()
    if start_twist is None:
        start_twist = (0.0, 0.0, 0.0)
    if start_pose is None:
        start_pose = (0.0, 0.0, 0.0)

    (pxl, pxr, fx) = lr_model[0]
    (pyl, pyr, fy) = lr_model[1]
    (pol, por, fo) = lr_model[2]

    (vxb, vyb, omegab) = start_twist
    (xb, yb, thetab) = start_pose

    # robot frame velocities
    vxr = math.cos(thetab) * vxb + math.sin(thetab) * vyb
    vyr = -math.sin(thetab) * vxb + math.cos(thetab) * vyb

    t = lrs[0]['t']
    path = [{'t': t, 'pose': (xb, yb, thetab), 'twist': (vxb, vyb, omegab)}]
    for i in range(1, len(lrs)):
        # calculate motion of point in the base frame using dynamic model
        t = lrs[i]['t']
        dt = t - lrs[i-1]['t']
        left = 0.5 * (lrs[i-1]['left'] + lrs[i]['left'])
        right = 0.5 * (lrs[i-1]['right'] + lrs[i]['right'])

        # apply the dynamic model for twist
        omegabOld = omegab
        omegab += dt * (left * pol + right * por - fo * omegab)

        vxbOld = math.cos(thetab) * vxr - math.sin(thetab) * vyr
        vybOld = math.sin(thetab) * vxr + math.cos(thetab) * vyr
        thetab += dt * 0.5 * (omegab + omegabOld)

        vxr += dt * (left * pxl + right * pxr - fx * vxr)
        vyr += dt * (left * pyl + right * pyr - fy * vyr)
        vxb = math.cos(thetab) * vxr - math.sin(thetab) * vyr
        vyb = math.sin(thetab) * vxr + math.cos(thetab) * vyr

        # integrate using previous values for pose
        xb += dt * 0.5 * (vxb + vxbOld)
        yb += dt * 0.5 * (vyb + vybOld)

        path.append([{'t': t, 'pose': (xb, yb, thetab), 'twist': (vxb, vyb, omegab)}])
        #print(fstr({'t': t, 'xb': xb, 'yb': yb, 'theta': thetab, 'left': left, 'right': right, 'vxr': vxr, 'vyr': vyr, 'vxb': vxb, 'vyb': vyb, 'omegab': omegab}))
    return path
