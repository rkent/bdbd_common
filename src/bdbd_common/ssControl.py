# state-space control of motion equations

import numpy as np

'''
https://github.com/python-control/Slycot/issues/15

What we do for testing in python-control is to use the following
set of commands to install slycot:

sudo apt-get install gfortran liblapack-dev
git clone https://github.com/python-control/Slycot.git slycot
cd slycot
sudo python setup.py install
'''

import control
import math
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import default_lr_model, transform2d

class SsControl():

    def __init__(self,
        poles=None,
        lr_model=default_lr_model(),
        mmax=1.0
    ):
        self.lr_model = lr_model
        self.mmax = mmax
        # poles for state control model
        if poles is None:
            fact = 0.9
            base = -2.0
            poles = []
            for i in range(6):
                poles.append(base)
                base = base * fact
        self.poles = np.array(poles)

        (bxl, bxr, _) = lr_model[0]
        (bol, bor, _)= lr_model[2]

        self.B = np.array((
                (bxl, bxr),
                (0, 0),
                (bol, bor),
                (0, 0),
                (0, 0),
                (0, 0)
        ))

    def __call__(self, dt, pose_m, twist_r, pp):
        (bxl, bxr, qx) = self.lr_model[0]
        (bol, bor, qo)= self.lr_model[2]

        zero3 = (0.0, 0.0, 0.0)
        frame_m = zero3
        # control algorithm
        pose_p = transform2d(pose_m, frame_m, pp.frame_p)
        vv = pp.nextPlanPoint(dt, pose_p)
        print(fstr({'vv': vv}))
        #if vv['fraction'] > 0.999:
        #    break

        # vv determines the normed frame for linearized control. Current pose
        # must be represented in this frame for control calculations. The frame definition
        # is the vv wheels in map frame
        frame_n = transform2d(vv['point'], pp.frame_p, frame_m)
        wheel_m = transform2d(pp.wheel_r, pose_m, frame_m)
        wheel_n = transform2d(wheel_m, frame_m, frame_n)
        theta_n = wheel_n[2]
        print(fstr({'frame_n': frame_n, 'wheel_n': wheel_n, 'pp.frame_p': pp.frame_p}))

        # twist in robot frame
        vxr_r = twist_r[0]
        vyr_r = twist_r[1]
        omega_r = twist_r[2]

        # robot (measurement point) twist in normed (vv) coordinates
        vxr_n = vxr_r * math.cos(theta_n) - vyr_r * math.sin(theta_n)
        vyr_n = vxr_r * math.sin(theta_n) + vyr_r * math.cos(theta_n)
        # wheel
        vxr_w = vxr_n
        vyw_n = vyr_n - pp.dwheel * omega_r

        omega0 = vv['omega']
        s0 = vv['v']
        # Note that "rho" is always positive, but "kappa" has the same sign as omega.
        if vv['kappa'] is None:
            sdot = 0.0
            odot = vv['direction'] * vv['d_vhat_dt'] / pp.rhohat
        else:
            sdot = vv['d_vhat_dt'] / (1.0 + pp.rhohat * abs(vv['kappa']))
            odot = vv['kappa'] * sdot

        eps = np.array([
            [vxr_w - vv['v']],
            [vyw_n],
            [omega_r - vv['omega']],
            [wheel_n[0]],
            [wheel_n[1]],
            [wheel_n[2]]
        ])

        sum = 0.0
        for err in eps:
            sum += err**2
        rms_err = math.sqrt(sum / len(eps))

        print(fstr({'eps': eps, 'rms_err': rms_err, 'vxw_n': vyw_n}))

        # the base frame (zero_map at start of plan) is the robot location. But the path
        # plan is in plan coordinates, with 0 being the wheel start.

        # RKJ 2021-01-12 p 72
        R0 = (
                (bol * (sdot + qx * s0) - bxl * (odot + qo * omega0)) /
                (bol * bxr - bxl * bor)
        )
        L0 = (
                (bor * (sdot + qx * s0) - bxr * (odot + qo * omega0)) /
                (bor * bxl - bxr * bol)
        )

        # small s0 values lead to an unstable control calculation
        s0 = max(s0, .04)

        A = np.array((
                (-qx, 0, 0, 0, 0, -omega0 * s0),
                (omega0, 0, s0, 0, 0, -qx * s0),
                (0, 0, -qo, 0, 0, 0),
                (1, 0, 0, 0, 0, 0),
                (0, 1, 0, 0, 0, 0),
                (0, 0, 1, 0, 0, 0)
        ))
        Kr = control.place_varga(A, self.B, self.poles)
        print(gstr({'Kr': Kr}))
        print(gstr({'eps * Kr': np.squeeze(eps) * np.asarray(Kr)}))
        lrs = -Kr * eps
        #print({'lrs': np.squeeze(lrs)})
        corr_left = lrs[0][0]
        corr_right = lrs[1][0]

        #corr_left = 0.0
        #corr_right = 0.0
        ctl_left = float(corr_left + L0)
        ctl_right = float(corr_right + R0)
        # correct for limits
        maxctl = max(abs(ctl_left), abs(ctl_right))
        if maxctl > self.mmax:
            ctl_left *= self.mmax / maxctl
            ctl_right *= self.mmax / maxctl
        lr = (ctl_left, ctl_right)
        state = {
            'eps': eps,
            'rms_err': rms_err,
            'lr_corr': (corr_left, corr_right),
            'vv': vv,
            'lr_base': (L0, R0)
        }
        return (lr, state)

