"""Example outer-loop controller, independent of CSim's physical step.

An educational PD position / quaternion attitude controller, not a flight-safety
controller. Assumes moderate tilt, known mass and no external disturbance.
"""
import math

def dot(a,b): return sum(x*y for x,y in zip(a,b))
def cross(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def unit(a):
    n=math.sqrt(dot(a,a))
    return [x/n for x in a]
def multiply(a,b):
    w,x,y,z=a; v,i,j,k=b
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]
def attitude(z):
    """Desired body Z, with body X aligned to the projection of world X."""
    y=unit(cross(z,[1,0,0])); x=cross(y,z)
    r=[[x[i],y[i],z[i]] for i in range(3)]
    # Moderate-tilt example keeps trace positive (acceleration limits below).
    w=math.sqrt(1+r[0][0]+r[1][1]+r[2][2])/2
    return [w,(r[2][1]-r[1][2])/(4*w),(r[0][2]-r[2][0])/(4*w),(r[1][0]-r[0][1])/(4*w)]

def ctbr(state,position,velocity,acceleration,mass,gravity,swing_gain=0, *, position_gain=2.0, velocity_gain=2.5, attitude_gain=5.0,
         acceleration_limits=(3.,3.,4.), rate_limit=3., minimum_thrust_ratio=.1):
    a=[acceleration[i]+position_gain*(position[i]-state['position_W'][i])
       +velocity_gain*(velocity[i]-state['velocity_W'][i]) for i in range(3)]
    if swing_gain:
        q=state['cable_direction_W']
        qdot=cross(state['cable_angular_velocity_W'],q)
        for i in range(2): a[i]+=swing_gain*qdot[i]
    a=[max(-limit,min(limit,value)) for value,limit in zip(a,acceleration_limits)]
    a[2]+=gravity
    desired=attitude(unit(a)); q=state['q_WB']
    error=multiply([q[0],-q[1],-q[2],-q[3]],desired)
    sign=1 if error[0]>=0 else -1
    rates=[max(-rate_limit,min(rate_limit,2*attitude_gain*sign*x)) for x in error[1:]]
    w,x,y,z=q; body_z=[2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)]
    thrust=max(minimum_thrust_ratio*mass*gravity,mass*dot(a,body_z))
    return thrust,rates

def reference(task,t):
    if task!='tracking': return [0,0,5],[0,0,0],[0,0,0]
    # Smooth periodic path, starts at rest at the initial position.
    omega=.35; angle=omega*t
    return ([1-math.cos(angle),.5*(1-math.cos(2*angle)),5],
            [omega*math.sin(angle),omega*math.sin(2*angle),0],
            [omega**2*math.cos(angle),2*omega**2*math.cos(2*angle),0])
