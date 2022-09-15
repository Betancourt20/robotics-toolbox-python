#!/usr/bin/env python

from math import pi, sin, cos
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3


class Jaco2(DHRobot):
    """
    Class that models a  Kinova Jaco Gen2 manipulator (spherical)

    :param symbolic: use symbolic constants
    :type symbolic: bool

    ``Jaco()`` is an object which models a Kinova Jaco robot and
    describes its kinematic characteristics using standard DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.Jaco_spherical()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. note::
        - SI units are used.

    :references:
        - "DH Parameters of Jaco" obtained from .https://github.com/Kinovarobotics/kinova-ros/

    :seealso: :func:`Mico`
    """

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym
            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi
            zero = 0.0

        deg = pi / 180
        # robot length values (metres)
        D1 = 0.2755
        D2 = 0.4100
        D3 = 0.2073
        D4 = 0.1038
        D5 = 0.1038
        D6 = 0.1600
        e2 = 0.0098

        L = [
            # Base
            RevoluteDH(
                d=D1,  # link length (Dennavit-Hartenberg notation)
                a=0,  # link offset (Dennavit-Hartenberg notation)
                alpha=pi/2,  # link twist (Dennavit-Hartenberg notation)
                offset=pi,
                m=0.46784,  # mass of link
                # distance of ith origin to center of mass [x,y,z]
                # in link reference frame
                r=[0, 0, 0.1255],
                # inertia tensor of link with respect to
                # center of mass  I = [L_xx, L_yy, L_zz,
                #                     L_xy, L_yz, L_xz]
                I=[9.5127e-04, 9.5127e-04, 3.7427e-04, 0, 0, 0],
                qlim=[-2*pi, 2*pi]  # minimum and maximum joint angle
            ),
            # Shoulder
            RevoluteDH(
                d=0,
                a=D2,
                alpha=pi,
                offset=-pi/2,
                m=0.7477,
                r=[0, -0.002, -0.0605],
                I=[0.0015, 0.0015, 5.9816e-04, 0, 0, 0],
                qlim=[47*deg, 313*deg]
            ),

            # arm
            RevoluteDH(
                d=-e2,
                a=0,
                alpha=pi/2,
                offset=-pi/2,
                m=0.99,
                r=[0, -0.2065, -0.01],
                I=[0.0105, 7.9200e-04, 0.0105, 0, 0, 0],
                qlim=[19*deg, 341*deg]
            ),
            # forearm
            RevoluteDH(
                d=-(D3+D4),
                a=0,
                alpha=pi/2,
                m=0.6763,
                r=[0, 0.081, -0.0086],
                I=[0.0014, 3.0433e-04, 0.0014, 0, 0, 0],
                qlim=[-2*pi, 2*pi]
            ),

            # wrist_spherical_1
            RevoluteDH(
                d=0,
                a=0,
                alpha=pi/2,
                m=0.463,
                r=[0, 0.0028848942, -0.0541932613],
                I=[4.3213e-04, 4.3213e-04, 9.2600e-05, 0, 0, 0],
                qlim=[30*deg, 330*deg]
            ),

            # wrist_spherical_2
            RevoluteDH(
                d=-(D5+D6),
                a=0,
                alpha=pi,
                offset=pi/2,
                m=0.463,
                r=[0, 0.0497208855, -0.0028562765],
                I=[4.3213e-04, 9.2600e-05, 4.3213e-04, 0, 0, 0],
                qlim=[-2*pi, 2*pi]
            )
        ]

        super().__init__(
            L,
            name='Jaco2',
            manufacturer='Kinova',
            keywords=('symbolic',)
        )
        self.qr =  np.r_[180, 180, 180, 0, 0, 180]* deg # reset position
        self.qz = np.r_[0, 0, 0, 0, 0, 0] # zero angles
        self.qn = np.r_[0, 180, 90, 0, 135, 0]*deg  # Position that make the worst-configuration for the inertial matrix
        self.qh =  np.r_[0, 2.9, 1.3, -2.07, 1.4, 0]  # Home position by default
        
        self.addconfiguration("qz", self.qz ) 
        self.addconfiguration("qr", self.qr) 
        self.addconfiguration("qn", self.qn )
        self.addconfiguration("qh", self.qh) 


if __name__ == '__main__':    # pragma nocover

    jaco2 = Jaco2(symbolic=False)
    print(jaco2)
