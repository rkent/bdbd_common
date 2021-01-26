# Common geometry methods

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from math import sin, cos, pi, sqrt, atan2, tan, copysign
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, PointStamped, Pose
from bdbd_common.utils import fstr

D_TO_R = pi / 180. # degrees to radians
TWOPI = 2. * pi
HALFPI = pi / 2.0

def poseDistance(pose1, pose2):
    # calculate the distance between two PoseStamped types in the same frame
    if pose1.header.frame_id != pose2.header.frame_id:
        raise RuntimeError('poses must be in the same frame')
    p1 = pose1.pose.position
    p2 = pose2.pose.position
    return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

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

def oneArcPath(x0, y0, theta):
    ''' calculate the arc to find the closest point to x,y with angle theta.
        See RKJ 01-21-2021
    '''
    px = .5 * (x0 * (1. + cos(theta)) + y0 * sin(theta))
    py = .5 * (x0 * sin(theta) + y0 * (1.-cos(theta)))
    if px > 0.0:
        kappa = sin(theta) / px
        rho = abs(1./kappa) if kappa != 0.0 else None
        arc = {
            'start': (0.0, 0.0, 0.0),
            'end': (px, py, theta),
            'center': (0.0, 1./kappa) if kappa != 0.0 else None,
            'radius': rho,
            'angle': theta,
            'length': abs(rho * theta) if rho is not None else sqrt(px**2 + py**2),
            'kappa': kappa
        }
    else:
        # just rotate in place
        arc = {
            'start': (0.0, 0.0, 0.0),
            'end': (0.0, 0.0, theta),
            'center': (0.0, 0.0, 0.0),
            'radius': 0.0,
            'angle': theta,
            'length': 0.0,
            'kappa': None

        }
    return arc

def oneArcPathOld(x, y, beta):
    ''' calculate the arc to find the closest point to x,y with angle beta.
        See RKJ 12-18-20 p 65
    '''
    # constrain beta to +/- pi
    beta = (beta + pi) % TWOPI - pi
    denom = y * tan(beta) + x
    if denom == 0.0:
        kappa = 1.e10
    else:
        kappa = tan(beta) / denom
    if kappa == 0.0:
        kappa = 1.e-10
    
    arc = {
        'start': (0.0, 0.0, 0.0),
        'end': (sin(beta) / kappa, (1. - cos(beta)) / kappa, beta),
        'center': (0.0, 1./kappa),
        'radius': abs(1./kappa),
        'angle': beta,
        'length': abs(beta/kappa),
        'kappa': kappa
    }
    return arc

def threeSegmentPath( x, y, phi, rho):
    #print('threeSegmentPath: ' + fstr({'x': x, 'y': y, 'phi': phi, 'rho': rho},'8.5f'))
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
    fc_ccw = (x - rho * sin(phi), y + rho * cos(phi))
    fc_cw = (x + rho * sin(phi), y - rho * cos(phi))

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
            theta = atan2(b, a)
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
                    psi = math.acos(2. * rho / sqrt(tsq))
                    alpha = psi - theta if start_dir == 1 else psi + theta
                    beta = pi/ 2. - alpha
                s = sqrt(ssq)
                beta = beta % TWOPI

                E = [rho * sin(beta), rho * (1. - cos(beta))] # transition from start circle to line
                if start_dir == -1:
                    E[1] = -E[1]
                # F is transition from line to final circle See RKJ 2020-09-22
                if start_dir == 1:
                    if end_dir == 1:
                        F = [D[0] + rho * sin(beta), D[1] - rho * cos(beta)]
                    else:
                        F = [D[0] - rho * sin(beta), D[1] + rho * cos(beta)]
                else:
                    if end_dir == 1:
                        F = [D[0] - rho * sin(beta), D[1] - rho * cos(beta)]
                    else:
                        F = [D[0] + rho * sin(beta), D[1] + rho * cos(beta)]

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
                l0 = rho * beta
                l1 = s
                l2 = rho * gamma
                solution = {
                    'dir': (start_dir, end_dir),
                    'l0': l0,
                    'l1': l1,
                    'l2': l2,
                    'length': l0 + l1 + l2,
                    'E': E,
                    'F': F,
                    'beta': beta,
                    'gamma': gamma
                }
                solutions.append(solution)

    # determine the best solution
    #for solution in solutions:
    #    print(fstr(solution))
    solution = solutions[0]
    for i in range(1, len(solutions)):
        #print(fstr(solutions[i]))
        if solutions[i]['length'] < solution['length']:
            solution = solutions[i]

    # return a path plan
    first_arc = {
        'start': (0.0, 0.0, 0.0),
        'end': (solution['E'][0], solution['E'][1], solution['beta'] * solution['dir'][0]),
        'center': sc_ccw if solution['dir'][0] == 1 else sc_cw,
        'radius': rho,
        'angle': solution['beta'] * solution['dir'][0],
        'length': solution['l0'],
        'kappa': solution['dir'][0] / rho, 
    }
    second_segment = {
        'start': first_arc['end'],
        'end': (solution['F'][0], solution['F'][1], solution['beta'] * solution['dir'][0]),
        'length': solution['l1'],
        'kappa': 0.0
    }
    third_arc = {
        'start': second_segment['end'],
        'end': (x, y, second_segment['end'][2] + solution['gamma'] * solution['dir'][1]),
        'center': fc_ccw if solution['dir'][1] == 1 else fc_cw,
        'radius': rho,
        'angle': solution['gamma'] *  solution['dir'][1],
        'length': solution['l2'],
        'kappa': solution['dir'][1] / rho
    }

    motion_plan = []
    if first_arc['angle'] != 0.0:
        motion_plan.append(first_arc)

    if (
        second_segment['start'][0] != second_segment['end'][0] or
        second_segment['start'][1] != second_segment['end'][1]
    ):
        motion_plan.append(second_segment)

    if third_arc['angle'] != 0.0:
        motion_plan.append(third_arc)

    #print('1 of 3 segments: ' + fstr(first_arc,'8.5f'))
    #print('2 of 3 segments: ' + fstr(second_segment,'8.5f'))
    #print('3 of 3 segments: ' + fstr(third_arc,'8.5f'))
    return motion_plan

def ccwPath(phi, x, y):
    '''
        determine a path from (0, 0) to a point at (x, y), with ending orientation phi from current
        orientation. Path consists of two circles with the same radius, intersecting at a tangent.
        See RKJ notebook circa 2020-09-12. This solution is just for initial CCW rotation. Negate
        phi, Y, ey,  beta for CW rotation.
    '''
    A = 1.0 - cos(phi)
    B = y * (1. + cos(phi)) - x * sin(phi)
    C = - (x**2 + y**2) / 2.

    # alternate form of quadratic equation, allows zero A
    try:
        rho = 2. * C / (-B - sqrt(B**2 - 4. * A * C))
    except ZeroDivisionError:
        return ccwPath(phi + 1.e-6, x, y)

    #   Calculate diagram values

    # x distance from target point to second circle center
    g = rho * sin(phi)
    # x distance between circle centers
    a = x + g
    # y distance between circle centers.
    b = rho * (1. + cos(phi)) - y
    # arc angle on first circle. Also orientation of robot at intersection point.
    beta = (atan2(a , b) + TWOPI) % TWOPI
    # intersection coordinates of circle tangents
    e = [rho * sin(beta), rho * (1. - cos(beta))]
    length = rho * beta
    gamma = (beta - phi + TWOPI) % TWOPI
    returns = {}
    for v in ['rho', 'beta', 'e', 'gamma', 'a', 'b', 'length']:
        returns[v] = eval(v)
    return returns

def nearPath(x, y, phi):
    # compatibility-mostly: return the shortest twoArcPlan
    plans = twoArcPath(x, y, phi)
    #print(fstr(plans))
    lengths = []
    for plan in plans:
        lengths.append(plan[0]['length'] + plan[1]['length'])
    return plans[0] if lengths[0] < lengths[1] else plans[1]

def twoArcPath(x, y, phi):
    #print('twoArcPath:\n' + fstr({'x': x, 'y': y, 'phi':phi}))
    # returns path plans for two intersecting arcs.
    phi = (phi + pi) % (2.0 * pi) - pi
    # see ccw path
    paths = (ccwPath(phi, x, y),ccwPath(-phi, x, -y))
    paths[1]['beta'] *= -1
    paths[1]['e'][1] *= -1

    plans = []
    for path in paths:
        dir = 1.0 if path['beta'] > 0.0 else -1.0
        first_arc = {
            'start': (0.0, 0.0, 0.0),
            'end': (path['e'][0], path['e'][1], path['beta']),
            'center': (0.0, dir * path['rho']),
            'radius': path['rho'],
            'angle': path['beta'],
            'length': abs(path['beta'] * path['rho']),
            'kappa': dir * (1. / path['rho'])
        }
        second_arc = {
            'start': first_arc['end'],
            'end': (x, y, phi),
            'center': (path['a'], dir * (path['rho'] - path['b'])),
            'radius': path['rho'],
            'angle': -dir * path['gamma'],
            'length': abs(path['rho'] * path['gamma']),
            'kappa': -dir * (1. / path['rho'])
        }
        #print(fstr({'1 of 2 arcs': first_arc}))
        #print(fstr({'2 of 2 arcs': second_arc}))
        plans.append((first_arc, second_arc))
    return plans

def b_to_w(base_pose, dwheel):
    # given the base pose, return the wheel pose
    theta = base_pose[2]
    return [
        base_pose[0] + dwheel * cos(theta),
        base_pose[1] + dwheel * sin(theta),
        theta
    ]

def w_to_b(wheel_pose, dwheel):
    theta = wheel_pose[2]
    return [
        wheel_pose[0] - dwheel * cos(theta),
        wheel_pose[1] - dwheel * sin(theta),
        theta
    ]

def lrEstimate(path, lr_model, start_twist, dt=0.025, left0 = 0.0, right0 = 0.0):
    # TODO: not working 2020-09-29
    # estimate robot left, right values to achieve a certain path, stopping with zero twist
    (pxl, pxr, fx) = lr_model[0]
    (pol, por, fo) = lr_model[2]
    beta = path['beta']
    rho = path['rho']
    gamma = path['gamma']
    length1 = abs(beta * rho)
    length2 = abs(gamma * rho)
    #print(fstr({'length1': length1, 'length2': length2}))

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
    o_1 = copysign(v_1 / rho, beta)
    o_2 = -o_1
    #print(fstr({'v_1': v_1, 'o_1': o_1, 'o_2:': o_2}))

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
        #print(fstr({'t': t, 'left': left, 'right': right, 's': s, 'v': v, 'omega': omega}))

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
    return ([pxl, pxr, fx], [pyl, pyr, fy], [pol, por, fo])

class Motor:
        # return motor left, right for a given speed, rotation
        # See RKJ 2020-09-14 pp 25
    def __init__(self, lr_model=default_lr_model(), mmax = 1.0):
        pxl, pxr, fx = lr_model[0]
        pol, por, fo = lr_model[2]
        self.a = pxl / fx
        self.b = pxr / fx
        self.c = pol / fo
        self.d = por / fo
        self.denom_left = self.b * self.d - self.a * self.c
        self.denom_right = self.b * self.c - self.a * self.d
        self.mmax = mmax

    def __call__(self, v, omega):
        left = (v * self.d - omega * self.b) / self.denom_left
        right = (v * self.c - omega * self.a) / self.denom_right
    
        scale = 1.0
        if abs(left) > self.mmax:
            scale = abs(left / self.mmax)
        if abs(right) > self.mmax:
            scale = max(scale, abs(right / self.mmax))
        return (left / scale, right / scale)

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
    vxr = cos(thetab) * vxb + sin(thetab) * vyb
    vyr = -sin(thetab) * vxb + cos(thetab) * vyb

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

        vxbOld = cos(thetab) * vxr - sin(thetab) * vyr
        vybOld = sin(thetab) * vxr + cos(thetab) * vyr
        thetab += dt * 0.5 * (omegab + omegabOld)

        vxr += dt * (left * pxl + right * pxr - fx * vxr)
        vyr += dt * (left * pyl + right * pyr - fy * vyr)
        vxb = cos(thetab) * vxr - sin(thetab) * vyr
        vyb = sin(thetab) * vxr + cos(thetab) * vyr

        # integrate using previous values for pose
        xb += dt * 0.5 * (vxb + vxbOld)
        yb += dt * 0.5 * (vyb + vybOld)

        path.append([{'t': t, 'pose': (xb, yb, thetab), 'twist': (vxb, vyb, omegab)}])
        #print(fstr({'t': t, 'xb': xb, 'yb': yb, 'theta': thetab, 'left': left, 'right': right, 'vxr': vxr, 'vyr': vyr, 'vxb': vxb, 'vyb': vyb, 'omegab': omegab}))
    return path

def nearestLinePoint(start, end, point):
    # The nearest point on line segment start, end to point p
    # adapted from https://forum.unity.com/threads/how-do-i-find-the-closest-point-on-a-line.340058/

    # return the nearest point, and the fraction of distance from start to end of the point
    if start[0] == end[0] and start[1] == end[1]:
        return (start, 0.0)
    
    normSE = sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
    lineDir = ((end[0] - start[0]) / normSE, (end[1] - start[1]) / normSE)
    v = (point[0] - start[0], point[1] - start[1])
    d = v[0] * lineDir[0] + v[1] * lineDir[1]
    if d <= 0.0:
        return 0.0, start
    elif d > normSE:
        return 1.0, end
    else:
        c = start[0] + d * lineDir[0], start[1] + d * lineDir[1]
        return d / normSE, c

def nearestArcPoint(center, rho, thetaStart, alpha, point):
    #print(fstr((center, rho, thetaStart, alpha, point)))
    # returns the fraction along an arc, and the closest point on that arc,
    # to a point "point" Arc has center with radius rho, beginning
    # angle (relative to x-axis) of thetaStart
    # negative alpha means clockwise arc direction
    # See RKJ 2020-09-28 pp 40

    # constrain alpha to -2pi -> + 2pi. Negative angle means CW rotation through arc
    if alpha > 0.0:
        alpha = (alpha + TWOPI) % TWOPI
    else:
        alpha = - ((-alpha + TWOPI) % TWOPI)

    if (point[0] == center[0] and point[1] == center[1]) or alpha == 0.0:
        fraction = 0.0
        thetaI = thetaStart
    else:
        thetaEnd = thetaStart + alpha
        gamma = thetaStart + 0.5 * alpha
        beta = atan2(point[1] - center[1], point[0] - center[0])
        betaPrime = (beta - gamma + pi) % TWOPI - pi
        fractionPrime = betaPrime / alpha
        if fractionPrime > 0.5:
            fraction = 1.0
            thetaI = thetaEnd
        elif fractionPrime < -0.5:
            fraction = 0.0
            thetaI = thetaStart
        else:
            fraction = fractionPrime + 0.5
            thetaI = thetaStart + fraction * alpha

    closest = (center[0] + rho * cos(thetaI), center[1] + rho * sin(thetaI))
    return (fraction, closest)

def transform2d(poseA, frameA, frameB):
    # transform poseA from frameA to frameC.
    # See RKJ 2020-10-1 pp 42-43

    # these are in world frame M
    (AxM, AyM, AthetaM) = frameA
    (BxM, ByM, BthetaM) = frameB

    # input pose is in frame A
    (xA, yA, thetaA) = poseA

    # transform B origin from world frame M to A

    BxA = (BxM - AxM) * cos(AthetaM) + (ByM - AyM) * sin(AthetaM)
    ByA = (ByM - AyM) * cos(AthetaM) -(BxM - AxM) * sin(AthetaM)

    # translate point from A-relative to B-relative in A orientation
    xAB = xA - BxA
    yAB = yA - ByA

    # rotate point to B orientation
    theta = BthetaM - AthetaM
    xB = xAB * cos(theta) + yAB * sin(theta)
    yB = yAB * cos(theta) - xAB * sin(theta)
    thetaB = thetaA - theta
    return (xB, yB, thetaB) 

def pose2to3(pose2d):
    # convert a 2d pose (x, y, theta) to a Pose message
    pose3d = Pose()
    pose3d.position.x = pose2d[0]
    pose3d.position.y = pose2d[1]
    pose3d.orientation = array_to_q(quaternion_from_euler(0.0, 0.0, pose2d[2]))
    return pose3d

def pose3to2(pose3d):
    # convert 3D Pose message to 2D coordinates x, y, theta.
    theta = euler_from_quaternion(q_to_array(pose3d.orientation))[2]
    pose2d = (pose3d.position.x, pose3d.position.y, theta)
    return pose2d

class DynamicStep:
    # simple dynamic model of bdbd motion
    def __init__(self, lr_model=default_lr_model()):
        self.lr_model = lr_model
        print(fstr({'DynamicStep lr_model': self.lr_model}))

    def __call__(self, lr, dt, pose_m, twist_m):
        # apply lr=(left, right) over time dt, starting from pose=(x, y, theta) and twist=(vx, vy, omega)
        (x_m, y_m, theta_m) = pose_m
        (vx_m, vy_m, omega) = twist_m
        (left, right) = lr

        (pxl, pxr, fx) = self.lr_model[0]
        (pyl, pyr, fy) = self.lr_model[1]
        (pol, por, fo) = self.lr_model[2]

        # robot frame velocities
        vx_r = cos(theta_m) * vx_m + sin(theta_m) * vy_m
        vy_r = -sin(theta_m) * vx_m + cos(theta_m) * vy_m

        # dynamic model in robot coordinates
        omegaNew = omega + dt * (left * pol + right * por - fo * omega)
        vxNew_r = vx_r + dt * (left * pxl + right * pxr - fx * vx_r)
        vyNew_r = vy_r + dt * (left * pyl + right * pyr - fy * vy_r)

        # map (or base) coordinate values
        thetaNew_m = theta_m + dt * 0.5 * (omega + omegaNew)
        vxNew_m = cos(thetaNew_m) * vxNew_r - sin(thetaNew_m) * vyNew_r
        vyNew_m = sin(thetaNew_m) * vxNew_r + cos(thetaNew_m) * vyNew_r
        xNew_m = x_m + dt * 0.5 * (vx_m + vxNew_m)
        yNew_m = y_m + dt * 0.5 * (vy_m + vyNew_m)
        return ((xNew_m, yNew_m, thetaNew_m), (vxNew_m, vyNew_m, omegaNew), (vx_r, vy_r, omegaNew))

def lr_est(vx, omega, last_vx, last_omega, dt, lr_model=default_lr_model(), mmax=1.0):
    # estimate values for left, right using the dynamic model with limits RKJ 2020-10-26 p 52
    (pxl, pxr, fx) = lr_model[0]
    (pol, por, fo) = lr_model[2]

    delx = (vx - last_vx * (1.0 - dt * fx)) / dt
    #print('delx: {}'.format(delx))
    delo = (omega - last_omega * (1. - dt * fo)) / dt
    right = (pol * delx - pxl * delo) / (pol * pxr - pxl * por)
    left = (delx - pxr * right) / pxl

    # scale left, right if limits exceeded
    if abs(left) > mmax or abs(right) > mmax:
        if abs(left) > abs(right):
            # scale by left
            right = abs(mmax / left) * right
            left = mmax if left > 0.0 else -1.0
        else:
            # scale by right
            left = abs(mmax / right) * left
            right = mmax if right > 1.0 else -1.0
        new_vx = last_vx + dt * (pxl * left + pxr * right - fx * last_vx)
        new_omega = last_omega + dt * (pol * left + por * right - fo * last_omega)
    else:
        new_vx = vx
        new_omega = omega

    return (left, right, new_vx, new_omega)

