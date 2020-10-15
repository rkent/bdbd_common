from bdbd_common.geometry import threeSegmentPath, twoArcPath, HALFPI, TWOPI, \
    default_lr_model, pose3to2, nearestLinePoint, nearestArcPoint, transform2d

import math

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
        Cd=125.,
        Cp=1000.,
        Cj=0.0,
        Fp=1.0,
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
        self.dy_w = None
        self.psi = None
        self.kappamax = 1. / min_rho

        # parameters for the control model. See RKJ 2020-10-07 p 47
        self.Cd = Cd # sin(angle) to plan factor
        # .130 is the distance from base to center of rotation
        self.Cp = Cp # distance to plan factor
        self.Cj = Cj # rate of change of sin(angle) to plan factor
        self.Fp = Fp # velocity factor
        self.Fd = Fd # rate of change of velocity factor
        # parameters rarely changed
        self.lpclose = 0.05 # distance from destination to start using actual kappa instead of plan
        self.lpstart = 0.5 # fraction of plan length to stop considering expected speed from time
        self.evold = 0.0
        self.sinpold = 0.0
        self.tt = 0.0

        # the offset from lr_model base of zero-vy center of rotation RKJ 2020-09-30 p 41
        (pyl, pyr, fy) = self.lr_model[1]
        (pol, por, fo) = self.lr_model[2]
        self.dwheel = (fo / fy) * (pyr - pyl) / (por - pol)

        # the rotation center in robot coordinates
        self.wheel_r = (-self.dwheel, 0.0, 0.0)
        # the robot base (measurement point) relative to the center of rotation
        self.robot_w = (self.dwheel, 0.0, 0.0)

    def start(self, start_pose, end_pose):
        # develop a path plan from start_pose to end_pose (as Pose message format)
        self.start_pose = start_pose
        self.end_pose = end_pose

        # convert to 2D coordinates x, y, theta. The pose coordinates are in the frame which will
        # be considered the base frame for these calculations.
        self.frame_m = (0., 0., 0.) # this is the privileged frame that others are relative to
        self.start_m = pose3to2(start_pose)
        self.end_m = pose3to2(end_pose)
            
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
            return ()

        # select the best two segment solution, if it exists
        paths2a = twoArcPath(*self.end_p)
        path2a = None
        for p in paths2a:
            if p[0]['radius'] > self.min_rho:
                # pylint: disable=unsubscriptable-object
                if path2a is None or (p[0]['radius'] > self.min_rho and p[0]['radius'] < path2a[0]['radius']):
                    path2a = p

        path3a = threeSegmentPath(self.end_p[0], self.end_p[1], self.end_p[2], self.approach_rho)

        if path2a and (
             path2a[0]['radius'] > self.min_rho and path2a[0]['radius'] < self.approach_rho
        ):
            # use the 2 segment solution
            path3a = None
        else:
            # use the 3 segment solution
            path2a = None

        self.path = path2a or path3a

        # add lprime to path plan, used by velocity
        for seg in self.path:
            if 'radius' in seg:
                rho = seg['radius']
                seg['lprime'] = seg['length'] * (rho + self.rhohat) / rho
            else:
                seg['lprime'] = seg['length']

        return self.path

    def speedPlan(self, vhat0, vhatcruise, vhatn, u=0.50):
        '''
        Given the path plan, determine a speed plan. Speeds are in 'hat' transformed form,
        see RKJ 2020-09-23 p 33

        vhat0: starting speed
        vhatcruise: cruising speed
        vhatn: final speed
        u: time for speed transition
        '''

        if self.path is None:
            raise Exception("You need to call start to determine the path plan before calling speedPlan")

        # calculate the total path length in transformed coordinates
        lprime_sum = 0.0
        for seg in self.path:
            # pylint: disable=unsupported-membership-test,unsubscriptable-object
            if 'radius' in seg:
                lprime_sum += abs(seg['angle']) * (seg['radius'] + self.rhohat)
            else:
                lprime_sum += math.sqrt(
                    (seg['start'][0] - seg['end'][0])**2 +
                    (seg['start'][1] - seg['end'][1])**2
                )

        s_plan = None

        # See RKJ 2020-09-24 p36 for plan.
        # 1) Check for short path to target
        if vhat0 + vhatn != 0.0:
            dt = 2.0 * lprime_sum / (vhat0 + vhatn)
            if dt < 2.0 * u:
                s_plan = [
                    {
                        'start': 0.0,
                        'end': lprime_sum,
                        'vstart': vhat0,
                        'vend': vhatn,
                        'time': dt
                    }
                ]

        # 2) Ramp to a value, then ramp to vhatn
        if s_plan is None:
            vhatm = lprime_sum / u - 0.5 * (vhat0 + vhatn)
            if vhatm < vhatcruise:
                s_plan = [
                    {
                        'start': 0.0,
                        'end': lprime_sum / 2.0,
                        'vstart': vhat0,
                        'vend': vhatm,
                        'time': u
                    },
                    {
                        'start': lprime_sum / 2.0,
                        'end': lprime_sum,
                        'vstart': vhatm,
                        'vend': vhatn,
                        'time': u
                    }
                ]

        # 3) Add a length of vcruise in middle
        if s_plan is None:
            lprime_0 = 0.5 * u * (vhat0 + vhatcruise)
            lprime_n = 0.5 * u * (vhatn + vhatcruise)
            s_plan = [
                {
                    'start': 0.0,
                    'end': lprime_0,
                    'vstart': vhat0,
                    'vend': vhatcruise,
                    'time': u
                },
                {
                    'start': lprime_0,
                    'end': lprime_sum - lprime_n,
                    'vstart': vhatcruise,
                    'vend': vhatcruise,
                    'time': (lprime_sum - lprime_n - lprime_0) / vhatcruise
                },
                {
                    'start': lprime_sum - lprime_n,
                    'end': lprime_sum,
                    'vstart': vhatcruise,
                    'vend': vhatn,
                    'time': u
                }
            ]

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

        # get speed, s from s_plan. Modify time to last plan time
        if seg_index is None:
            seg_index = len(self.s_plan) - 1
            seg_time = self.s_plan[seg_index]['time']
            tend = 0.0
            for seg in self.s_plan:
                tend += seg['time']

        seg = self.s_plan[seg_index]
        vhat = seg['vstart'] + seg_time * (seg['vend'] - seg['vstart']) / seg['time']
        sprime = seg['start'] + 0.5 * seg_time * (seg['vstart'] + vhat)

        # get parameters from path plan
        s_sum = 0.0
        sprime_sum = 0.0
        v = None
        omega = None
        p = None
        for i in range(len(self.path)):
            seg = self.path[i]
            lprime = seg['lprime']

            if sprime_sum + lprime > sprime:
                # this is the correct segment
                frac = (sprime - sprime_sum) / lprime
                rho = seg['radius'] if 'radius' in seg else None
                if rho is None:
                    v = vhat
                    omega = 0.0
                    theta = seg['start'][2]
                    bx = seg['start'][0] + frac * (seg['end'][0] - seg['start'][0])
                    by = seg['start'][1] + frac * (seg['end'][1] - seg['start'][1])
                else:
                    v = vhat * rho / (rho + self.rhohat)
                    omega = math.copysign(v / rho, seg['angle'])
                    arc_angle = seg['angle'] * frac
                    theta = seg['start'][2] + arc_angle
                    if seg['angle'] > 0.0:
                        bx = seg['center'][0] + rho * math.sin(theta)
                        by = seg['center'][1] - rho * math.cos(theta)
                    else:
                        bx = seg['center'][0] - rho * math.sin(theta)
                        by = seg['center'][1] + rho * math.cos(theta)
    
                p = (bx, by, theta)
                break
            else:
                sprime_sum += lprime
                s_sum += seg['length']

        if v is None:
            p = self.path[-1]['end']
            if 'radius' in self.path[-1]:
                rho = self.path[-1]['radius']
                v = vhat * rho / (rho + self.rhohat)
                omega = math.copysign(v / rho, self.path[-1]['angle'])
            else:
                v = vhat
                omega = 0.0
                rho = None
            
        return {'time': tend, 'sprime': sprime, 'vhat': vhat, 'v': v, 'omega': omega, 'rho': rho, 'point': p}

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
        # get the wheel plan coordinates given the base coordinates. The base coordinates
        # are the origin of the robot frame, and we have the wheels position in that frame.
        self.tt += dt
        wheel_pose_p = transform2d(self.wheel_r, pose_m, self.frame_p)
        wheel_pose_m = transform2d(wheel_pose_p, self.frame_p, self.frame_m)

        # find closest point on plan
        (nearest_p, distance, vhat_near, kappa_near, lprime, lprime_sum) = self.nearestPlanPoint(wheel_pose_p)

        # get the orientation of the closest point in wheel coordinates. The wheel frame
        # is just the location of the wheel point in m frame
        nearest_w = transform2d(nearest_p, self.frame_p, wheel_pose_m)

        # variables for the control model. See RKJ 2020-10-07 p 47
        va = twist_r[0]
        oa = twist_r[2]
        kappaa = oa / va if va != 0.0 else 0.0
        vhata = va + abs(self.rhohat * oa)
        dy_w = -nearest_w[1]
        psi = -nearest_w[2]
        psi = (psi + math.pi) % TWOPI - math.pi
        sinp = math.sin(psi)
        dsinpdt = (sinp - self.sinpold) / dt
        self.sinpold = sinp

        # when we are near the end, use more actual kappa. This prevents rapid change between kappas if there are two nearby arcs at end.
        diff = lprime_sum - lprime
        a_p_ratio = max(0.0 , (self.lpclose - diff) / self.lpclose)

        # when we are near the beginning, rely more on t to determine vhat
        lp_frac = lprime / lprime_sum
        n_t_ratio = min(lp_frac / self.lpstart, 1.0)
        if n_t_ratio < 1.0:
            vv_tt = self.v(self.tt)
            vhat_tt = vv_tt['vhat']
            vhat_plan = n_t_ratio * vhat_near + (1.0 - n_t_ratio) * vhat_tt
        else:
            vhat_plan = vhat_near

        kappa_new = a_p_ratio * kappaa + (1.0 - a_p_ratio)* kappa_near -\
            self.Cp * dy_w - self.Cd * math.sin(psi) - self.Cj * dsinpdt
        kappa_new = min(self.kappamax, max(-self.kappamax, kappa_new))
        '''
        print(fstr({'a_p_ratio': a_p_ratio, 'kappaa': kappaa, 'kappa_near': kappa_near, 
            'Cp': self.Cp, 'dy_w': dy_w, 'Cd': self.Cd, 'psi': psi,
            'Cj': self.Cj, 'dsinpt': dsinpdt}))
        '''

        ev = vhata - vhat_plan
        devdt = (ev - self.evold) / dt
        self.evold = ev
        vhat_new = max(0.0, vhat_plan - self.Fp * ev - self.Fd * devdt)

        v_new = vhat_new / (1.0 + abs(self.rhohat * kappa_new))
        o_new = kappa_new * v_new

        self.nearest_p = nearest_p
        self.wheel_pose_p = wheel_pose_p
        self.wheel_pose_m = wheel_pose_m
        self.dy_w = dy_w
        self.va = va
        self.oa = oa
        self.psi = psi
        self.ev = ev
        self.vhat_new = vhat_new
        self.vhat_near = vhat_near
        self.vhat_plan = vhat_plan
        self.o_new = o_new
        self.v_new = v_new
        self.vhata = vhata
        self.kappaa = kappaa
        self.kappa_new = kappa_new
        self.kappa_near = kappa_near
        self.lp_frac = lp_frac
        self.dsinpdt = dsinpdt
        return (v_new, o_new)
