from bdbd_common.geometry import threeSegmentPath, twoArcPath, HALFPI, TWOPI, \
    default_lr_model, pose3to2, nearestLinePoint, nearestArcPoint, transform2d, lr_est, \
    oneArcPath

from bdbd_common.utils import fstr, gstr
import math

def static_plan(
    start_pose=(0.0, 0.0, 0.0),
    start_twist=(0.0, 0.0, 0.0),
    target_pose=(0.0, 0.0, 0.0),
    target_twist=(0.0, 0.0, 0.0),
    approach_rho=0.20, # radius of planned approach
    min_rho=0.10, # the smallest radius we allow in a path plan,
    cruise_v = 0.25,
    vslow=0.05,
    u=0.50,
    details=False,
    max_segments=3,
    vhat_start=None
):
    if details:
        print(fstr({
            's_pose': start_pose,
            't_pose': target_pose,
            's_twist': start_twist,
            't_twist': target_twist
        }))
    # estimate left, right to achieve the path
    pp = PathPlan(approach_rho=approach_rho, min_rho=min_rho)
    pp.start2(start_pose, target_pose, max_segments=max_segments)

    # calculate start vhat from start_twist
    if vhat_start is None:
        vhat_start = abs(start_twist[0]) + abs(start_twist[2] * pp.rhohat)
    pp.speedPlan(vhat_start, cruise_v, target_twist[0], u=u, vslow=vslow)
    if details:
        print('path_plan')
        for seg in pp.path:
            print(fstr(seg))
        print('speed_plan')
        for seg in pp.s_plan:
            print(fstr(seg))

    return pp

def segSpeed(lp, v0=0.0, vc=0.3, vn=0.0, u=0.25):
    # plan for speeds in a static path, with maximum speed slew rate
    # lp: total path length
    # v0, vn: start, finish speeds
    # vc: cruise speed
    # u: time allowed to slew from 0 to vc
    # See RKJ 2020-12-21
    if vc <= 0.:
        raise Exception('vc must be positive, non-zero')
    s_plan = []

    # try ramp to vc, then to vn
    S = None if u == 0.0 else vc / u
    d0 = abs(vc*vc - v0*v0) / (2*S) if S else None
    d2 = abs(vc*vc - vn*vn) / (2*S) if S else None
    print(fstr({'v0': v0, 'vn': vn, 'vc': vc, 'lp': lp, 'u': u, 'd0': d0, 'd2': d2}))

    if S and d0 + d2 < lp:
        d1 = (lp - d0 - d2)
        t1 = d1 / vc
        s_plan = [
            {
                'start': 0.0,
                'end': d0,
                'vstart': v0,
                'vend': vc,
                'time': (vc - v0) / S
            },
            {
                'start': d0,
                'end': d0 + d1,
                'vstart': vc,
                'vend': vc,
                'time': t1
            },
            {
                'start': d0 + d1,
                'end': lp,
                'vstart': vc,
                'vend': vn,
                'time': (vc - vn) / S
            }
        ]

    else:
        # try to increase to a speed vm, then decelerate to vn
        dp = (v0*v0 + vn*vn) / (2.0 * S)
        vm2 = S * (lp + dp)
        vm = math.sqrt(vm2)

        if vm > v0 and vm > vn:
            # Case 2a: t1 = 0, 2-segment ramp up then down
            d0 = (vm*vm - v0*v0) / (2.0 * S)
            s_plan = [
                {
                    'start': 0.0,
                    'end': d0,
                    'vstart': v0,
                    'vend': vm,
                    'time': (vm - v0) / S
                },
                {
                    'start': d0,
                    'end': lp,
                    'vstart': vm,
                    'vend': vn,
                    'time': (vm - vn) / S
                }
            ]
        else:
            # just ramp from v0 to vn
            s_plan = [
                {
                    'start': 0.0,
                    'end': lp,
                    'vstart': v0,
                    'vend': vn,
                    'time': 2 * lp / (v0 + vn) if (v0 + vn) != 0.0 else 0.0
                }
            ]

    return s_plan

class PathPlan():
    '''
    Manage movement from a start pose, twist to an end pose, twist

    Frames of reference:

    Inputs (including poses, twists, and lr_model) should be in a consistent, stationary frame
    of reference. Typically that would be the pose, twist measurement point in a 'world' or 'map'
    frame of reference. I'll call this the 'm' frame (for measurement). No suffix on variables
    typically means this frame of reference, or suffix m for clarity. 

    Frames of reference and their suffixes:
        m   'measure':  stationary frame that world measurements are valid
        p   'plan':     stationary frame used as origin of path plan. Wheel pose at start of plan
        r   'robot':    moving frame where lr_model is valid (ie, the measurement point)
        w   'wheels':   moving frame centered on wheel, ie center of rotation

    '''
    def __init__(
        self,
        lr_model=default_lr_model(),
        approach_rho=0.2, # radius of planned approach
        min_rho=0.05, # the smallest radius we allow in a path plan,
        rhohat=0.184,  # tradeoff between speed and omega, see RKJ 2020-09-23 p 33
        # control model parameters
        Cp=125.,
        Cd=20.,
        Cy=20.,
        Cj=5.0,
        Cyf=0.25,
        Fp=0.25,
        Fd=0.0
    ):
        self.s_plan = None
        self.path = None,
        self.lr_model = lr_model
        self.approach_rho = approach_rho
        self.min_rho = min_rho
        self.rhohat = rhohat
        self.v_new = 0.0
        self.o_new = 0.0
        self.lp_frac = 0.0
        self.dy_r = None
        self.psi = None
        self.kappamax = 1. / min_rho if min_rho != 0.0 else None
        self.lastt = 0.0
        self.vhatmin = -0.1

        # parameters for the control model. See RKJ 2020-10-07 p 47
        self.Cd = Cd # sin(angle) to plan factor
        self.Cp = Cp # distance to plan factor
        self.Cj = Cj # rate of change of sin(angle) to plan factor
        self.Cy = Cy
        self.Cyf = Cyf
        self.Fp = Fp # velocity factor
        self.Fd = Fd # rate of change of velocity factor
        # parameters rarely changed
        self.lpclose = 1.0 # distance from destination to start using actual kappa instead of plan
        self.lpstart = 0.5 # fraction of plan length to stop considering expected speed from time
        self.evold = 0.0
        self.sinpold = 0.0
        self.tt = 0.0
        self.dyold = 0.0
        self.dydt_old = 0.0

        # the offset from lr_model base of zero-vy center of rotation RKJ 2020-09-30 p 41
        (pyl, pyr, fy) = self.lr_model[1]
        (pol, por, fo) = self.lr_model[2]
        self.dwheel = (fo / fy) * (pyr - pyl) / (por - pol)

        # the rotation center in robot coordinates
        self.wheel_r = (-self.dwheel, 0.0, 0.0)
        # the robot base (measurement point) relative to the center of rotation
        self.robot_w = (self.dwheel, 0.0, 0.0)
        self.frame_m = (0., 0., 0.) # this is the privileged frame that others are relative to

    def start(self, start_pose, end_pose):
        # develop a path plan from start_pose to end_pose (as Pose message format)
        self.start_pose = start_pose
        self.end_pose = end_pose

        # convert to 2D coordinates x, y, theta. The pose coordinates are in the frame which will
        # be considered the base frame for these calculations.
        start_m = pose3to2(start_pose)
        end_m = pose3to2(end_pose)
        return self.start2(start_m, end_m)

    def start2(self, start_m, end_m, max_segments=3):
        # _m here means the model frame, whose origin is the path starting point at the measurement point
        self.start_m = start_m
        self.end_m = end_m

        # at the start, the robot base is at start_m, so that is the origin of the robot frame
        self.wheelstart_m = transform2d(self.wheel_r, self.start_m, self.frame_m)

        # the path plan will be done in the wheel coordinates
        self.frame_p = self.wheelstart_m[:]
        self.start_p = (0., 0., 0.)

        # a frame centered on robot base is the same as the base point in m frame
        self.wheelend_m = transform2d(self.wheel_r, self.end_m, self.frame_m)
        #print(fstr({'start_m': self.start_m, 'end_m': self.end_m, 'wheelstart_m': self.wheelstart_m, 'wheelend_m': self.wheelend_m}))
        self.end_p = transform2d(self.wheelend_m, self.frame_m, self.frame_p)

        if self.start_m[0] == self.end_m[0] and self.start_m[1] == self.end_m[1] and self.start_m[2] == self.end_m[2]:
            # null request
            return []

        # determine a path plan given 2d coordinates
        
        # select the shortest two segment solution
        paths2a = twoArcPath(*self.end_p)
        el0 = paths2a[0][0]['length'] + paths2a[0][1]['length']
        el1 = paths2a[1][0]['length'] + paths2a[1][1]['length']
        path2a = paths2a[0] if el0 < el1 else paths2a[1]
        print(' ')

        if max_segments >= 3 and path2a[0]['radius'] > self.approach_rho:
            self.path = threeSegmentPath(*(self.end_p + (self.approach_rho,)))
        elif max_segments >= 2 and path2a[0]['radius'] > self.min_rho:
            self.path = path2a
        else:
            self.path = [oneArcPath(*self.end_p)]
            print('oneArcPath(' + fstr(self.end_p) + ')')

        for seg in self.path:
            print(fstr(seg))
        # add lprime to path plan, used by velocity
        for seg in self.path:
            if 'radius' in seg:
                rho = seg['radius']
                if rho == 0.0:
                    seg['lprime'] = abs(seg['angle'] * (rho + self.rhohat))
                else:
                    seg['lprime'] = seg['length'] * (rho + self.rhohat) / rho
            else:
                seg['lprime'] = seg['length']

        return self.path

    def speedPlan(self, vhat0, vhatcruise, vhatn, u=0.50, vslow=0.05):
        '''
        Given the path plan, determine a speed plan. Speeds are in 'hat' transformed form,

        vhat0: starting speed
        vhatcruise: cruising speed
        vhatn: final speed
        u: time for speed transition
        vslow: vhat at path plan transitions
        '''

        s_plan = []
        if self.path is None:
            raise Exception("You need to call start to determine the path plan before calling speedPlan")

        lprime_sum = 0.0
        for i in range(len(self.path)):
            seg = self.path[i]
            vend = vhatn if i == len(self.path) - 1 else vslow
            vstart = vhat0 if i == 0 else vslow
            #print(fstr({'seg': seg}))
            seg_plans = segSpeed(seg['lprime'], vstart, vhatcruise, vend, u)
            for seg_plan in seg_plans:
                seg_plan['start'] += lprime_sum
                seg_plan['end'] += lprime_sum
                s_plan.append(seg_plan)
            lprime_sum = seg_plans[-1]['end']

        self.s_plan = s_plan
        return s_plan

    def v(self, tend):
        # return the expected position and velocities at time tend from plan start.
        # these are all in wheel coordinates

        if self.s_plan is None or self.path is None:
            raise Exception('speed or path plan missing')

        # determine the speed plan segment
        tt = 0.0
        seg_time = None
        seg_index = None
        rho = None
        for i in range(len(self.s_plan)):
            seg = self.s_plan[i]
            if tend > tt + seg['time']:
                tt += seg['time']
                continue
            seg_time = tend - tt
            seg_index = i
            break

        if seg_index is None:
            # use the end of the plan
            seg_index = len(self.s_plan) - 1
            seg_time = self.s_plan[seg_index]['time']
            tend = 0.0
            for seg in self.s_plan:
                tend += seg['time']

        seg = self.s_plan[seg_index]
        d_vhat_dt = (seg['vend'] - seg['vstart']) / seg['time'] if seg['time'] else 0.0
        vhat = seg['vstart'] + seg_time * d_vhat_dt
        sprime = seg['start'] + 0.5 * seg_time * (seg['vstart'] + vhat)

        # get parameters from path plan
        s_sum = 0.0
        sprime_sum = 0.0
        v = None
        omega = None
        direction = 1.0
        p = None
        for i in range(len(self.path)):
            seg = self.path[i]
            lprime = seg['lprime']
            #print(fstr({'lprime': lprime}))

            if sprime_sum + lprime > sprime and v is None:
                # this is the correct segment
                frac = (sprime - sprime_sum) / lprime
                rho = seg['radius'] if 'radius' in seg else None
                kappa = seg['kappa']
                if rho is None:
                    v = vhat
                    omega = 0.0
                    theta = seg['start'][2]
                    bx = seg['start'][0] + frac * (seg['end'][0] - seg['start'][0])
                    by = seg['start'][1] + frac * (seg['end'][1] - seg['start'][1])
                else:
                    v = vhat * rho / (rho + self.rhohat)
                    direction = math.copysign(1.0, seg['angle'])
                    omega = math.copysign(vhat / (rho + self.rhohat), seg['angle'])
                    arc_angle = seg['angle'] * frac
                    theta = seg['start'][2] + arc_angle
                    if seg['angle'] > 0.0:
                        bx = seg['center'][0] + rho * math.sin(theta)
                        by = seg['center'][1] - rho * math.cos(theta)
                    else:
                        bx = seg['center'][0] - rho * math.sin(theta)
                        by = seg['center'][1] + rho * math.cos(theta)
    
                p = (bx, by, theta)
            sprime_sum += lprime
            s_sum += seg['length']

        if v is None:
            p = self.path[-1]['end']
            kappa = self.path[-1]['kappa']
            if 'radius' in self.path[-1]:
                rho = self.path[-1]['radius']
                v = vhat * rho / (rho + self.rhohat)
                omega = math.copysign(vhat / (rho + self.rhohat), self.path[-1]['angle'])
            else:
                v = vhat
                omega = 0.0
                rho = None

        return {
            'time': tend,
            'sprime': sprime,
            'vhat': vhat,
            'v': v,
            'omega': omega,
            'rho': rho,
            'kappa': kappa,
            'point': p,
            'd_vhat_dt': d_vhat_dt,
            'fraction': sprime / sprime_sum if sprime_sum != 0.0 else 1.0,
            'direction': direction
        }

    def nearestPlanPoint(self, wheel_pose_p):
        # find closest point on plan

        nearests = []
        for segment in self.path:
            if 'radius' in segment and segment['radius'] is not None:
                # an arc
                fraction, nearestPoint = nearestArcPoint(
                    segment['center'],
                    segment['radius'],
                    segment['start'][2] - math.copysign(HALFPI, segment['angle']), 
                    segment['angle'],
                    wheel_pose_p[:2]
                )
            else:
                # a line
                fraction, nearestPoint = nearestLinePoint(
                    segment['start'][:2],
                    segment['end'][:2],
                    wheel_pose_p[:2])
            # note nearest
            nearestPoint_p = (
                nearestPoint[0],
                nearestPoint[1],
                (segment['start'][2] + fraction * (segment['end'][2] - segment['start'][2]))
            )
            nearests.append((fraction, nearestPoint_p, segment['kappa']))

        best_distance = None
        best_segment = None
        best_fraction = None
        best_kappa = None
        best_nearest_p = None
        for i in range(len(nearests)):
            (fraction, nearestPoint_p, kappa) = nearests[i]
            d = math.sqrt((nearestPoint_p[0] - wheel_pose_p[0])**2 + (nearestPoint_p[1] - wheel_pose_p[1])**2)
            # also account for error in angle, see RKJ 2020-10-09 p 48
            d += self.rhohat * (1.0 - math.cos(nearestPoint_p[2] - wheel_pose_p[2]))
            if best_distance is None or d < best_distance:
                best_nearest_p = nearestPoint_p
                best_distance = d
                best_fraction = fraction
                best_segment = i
                best_kappa = kappa

        ### determine the speed at that point
        # Since the speed plan is in lprime, we need lprime at the point.
        lprime_sum = 0.0
        lprime = 0.0
        for i in range(len(self.path)):
            seg = self.path[i]
            if i == best_segment:
                lprime = lprime_sum + best_fraction * seg['lprime']
            lprime_sum += seg['lprime']

        # get the speed from the speed plan
        vhat = 0.0
        vsq = 0.0
        for seg in self.s_plan:
            if lprime >= seg['start'] and lprime <= seg['end']:
                if seg['time'] > 0.0:
                    vsq = seg['vstart']**2 + 2.0 * (lprime - seg['start']) * (seg['vend'] - seg['vstart']) / seg['time']
                if vsq > 0.0:
                    vhat = math.sqrt(vsq)
                else:
                    vhat = 0.0

        return (best_nearest_p, best_distance, vhat, best_kappa, lprime, lprime_sum)

    def controlStep(self, dt, pose_m, twist_r):
        if dt < 0.0001:
            return (self.v_new, self.o_new)

        ### control model
        self.tt += dt

        # find closest point on plan
        pose_p = transform2d(pose_m, self.frame_m, self.frame_p)
        near_wheel = self.nextPlanPoint(dt, pose_p)
        near_wheel_p = near_wheel['point']
        #print(fstr({'near_time:': self.lastt, 'near_wheel_p': near_wheel_p, 'pose_p': pose_p}))

        # The control model operates in the robot frame, relative to the plan. So we need the deviation
        # of the actual location from the plan location, in the robot frame.
        near_wheel_m = transform2d(near_wheel_p, self.frame_p, self.frame_m)
        # near_wheel_m is the current origin of the wheel frame for the nearest point
        near_robot_m = transform2d(self.robot_w, near_wheel_m, self.frame_m)
    
        # near_robot_m is the origin of the planned robot frame
        pose_r = transform2d(pose_m, self.frame_m, near_robot_m)

        # variables for the control model. See RKJ 2020-10-07 p 47
        va = twist_r[0]
        oa = twist_r[2]
        kappaa = oa / va if va != 0.0 else 0.0
        vhata = va + abs(self.rhohat * oa)
        dy_r = pose_r[1]
        psi = pose_r[2]
        psi = (psi + math.pi) % TWOPI - math.pi
        sinp = math.sin(psi)
        dsinpdt = (sinp - self.sinpold) / dt
        self.sinpold = sinp
        dydt_new = (dy_r - self.dyold) / dt
        self.dyold = dy_r
        dydt = self.Cyf * dydt_new + (1. - self.Cyf) * self.dydt_old
        self.dydt_old = dydt

        vhat_plan = near_wheel['vhat']
    
        denom = va + near_wheel['v']
        kappa_combo = (oa + near_wheel['omega']) / denom if denom != 0.0 else 0.0

        kappa_new = kappa_combo + \
            -self.Cp * dy_r - self.Cd * math.sin(psi) -  self.Cy * dydt - self.Cj * dsinpdt

        kappa_new = kappa_new if self.kappamax is None else min(self.kappamax, max(-self.kappamax, kappa_new))

        ev = vhata - vhat_plan
        devdt = (ev - self.evold) / dt
        self.evold = ev
        vhat_new = max(self.vhatmin, vhat_plan - self.Fp * ev - self.Fd * devdt)

        v_new = vhat_new / (1.0 + abs(self.rhohat * kappa_new))
        o_new = kappa_new * v_new

        self.near_wheel_p = near_wheel_p
        self.near_robot_m = near_robot_m
        self.pose_p = pose_p
        self.dy_r = dy_r
        self.va = va
        self.oa = oa
        self.psi = psi
        self.ev = ev
        self.vhat_new = vhat_new
        self.vhat_plan = vhat_plan
        self.o_new = o_new
        self.v_new = v_new
        self.vhata = vhata
        self.kappaa = kappaa
        self.kappa_new = kappa_new
        self.kappa_combo = kappa_combo
        self.kappa_plan = near_wheel['omega'] / near_wheel['v'] if near_wheel['v'] != 0.0 else 0.0
        self.dsinpdt = dsinpdt
        self.dydt = dydt
        self.lp_frac = near_wheel['fraction']
        return (v_new, o_new)

    def nextPlanPoint(self, dt, pose_p, corr_radius=1.0):

        # Adjust for theta error using RKJ notebook 2020-12-7
        #corr_radius = 0.10

        def distance(p1, p2):
            dtheta = (corr_radius * (1.0 - math.cos(p1[2] - p2[2])))**2
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + dtheta)

        def t_wp_rp(wp):
            wheel_m = transform2d(wp, self.frame_p, self.frame_m)
            robot_p = transform2d(self.robot_w, wheel_m, self.frame_p)
            #print(fstr({'t_wp_rp': (wp, wheel_m, robot_p, self.robot_w)}))
            return robot_p

        # determine the closest point on the plan to pose_p, moving forward dt seconds
        # RKJ 2020-10-21 p 50
        dtx = 0.75 * dt
        dt0 = 0.5 * dt
        t0 = self.lastt + dt0
        t1 = t0 + dtx
        t2 = t1 + dtx
        # transform2d(pp.robot_w, plan_w_m, pp.frame_m)
        pp0 = t_wp_rp(self.v(t0)['point'])
        pp1 = t_wp_rp(self.v(t1)['point'])
        pp2 = t_wp_rp(self.v(t2)['point'])
        d0 = distance(pose_p, pp0)
        d1 = distance(pose_p, pp1)
        d2 = distance(pose_p, pp2)
        #print(fstr((pose_p, self.v(t0), d0, d1, d2), fmat='8.5f'))
        deriv2 = d0 + d2 - 2.0 * d1 # proportional to second derivative
        if deriv2 <= 0.0:
            # this is a maximum, just pick the smallest edge. 0.0 is t0, 2.0 is t2
            tt = t0 if d0 < d2 else t2
        else:
            tau = (3.0 * d0 + d2 - 4.0 * d1) / (2.0 * deriv2)
            tt = t0 + tau * dtx
            #print('used deriv2, tt is {}'.format(tt))
        #print('nextPlanPoint ' + fstr((tt, pose_p, t0, d0, pp0, t1, d1, pp1, t2, d2, pp2), fmat='8.5f'))
        tt = max(t0, min(t2, tt))
        self.lastt = tt
        return self.v(tt)

if __name__ == '__main__':
    from bdbd_common.geometry import Motor
    pp = PathPlan()
    pathPlan = pp.start2((0., 0., 0.), (.3, .1, 0.))
    for seg in pathPlan:
        print(fstr(seg))
    
    speedPlan = pp.speedPlan(0.0, 0.25, 0.0)
    for seg in speedPlan:
        print(fstr(seg))

    dt = 0.02
    frac = 0.0
    tt = 0.0
    vps = []
    while frac < 0.999:
        vp = pp.v(tt)
        vps.append(vp)
        print(fstr(vp))
        frac = vp['fraction']
        tt += dt

    last_vx = 0.0
    last_omega = 0.0

    motor = Motor()
    tt = 0.0
    for vp in vps:
        (lm, rm) = motor(vp['v'], vp['omega'])
        (left, right, last_vx, last_omega) = lr_est(vp['v'], vp['omega'], last_vx, last_omega, dt)
        print(fstr({'tt': tt, 'lr': (left, right), 'lm': (lm, rm), 'last_vx': last_vx, 'last_omega': last_omega}))
        tt += dt

