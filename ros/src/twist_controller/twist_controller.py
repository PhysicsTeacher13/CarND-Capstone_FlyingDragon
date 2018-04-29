
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
    				accel_limit, wheel_radius, wheel_base,steer_ratio,
    				max_lat_accel, max_steer_angle):
        # TODO: Implement

        # make a YowController object to be used later for steering angle
        self.yaw_controller=YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)

        # make a PID object to be used later for throttle control (parameters can be tuned)
        kp=0.3
        ki=0.1
        kd=0
        mn_throttle=0
        mx_throttle=0.2
        self.pid_controller=PID(kp,ki,kd,mn_throttle,mx_throttle)

        # LOW PASS FILTER to get rid of velocity high frequency noise
        tau=0.5
        ts=0.02
        self.low_pass=LowPassFilter(tau,ts)

        # Save car parameters for further use
        self.vehicle_mass=vehicle_mass
        self.fuel_capacity=fuel_capacity
        self.brake_deadband=brake_deadband
        self.decel_limit=decel_limit
        self.accel_limit=accel_limit
        self.wheel_radius=wheel_radius

        self.previous_time=rospy.get_time()


    def control(self, current_vel, dwb_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dwb_enabled:
        	self.pid_controller.reset()
        	return 0, 0, 0
        
        current_vel=self.low_pass.filt(current_vel)
        steer=self.yaw_controller.get_steering(linear_vel,angular_vel,current_vel)

        vel_err=linear_vel- current_vel
        current_time=rospy.get_time()
        sample_time=current_time-self.previous_time
        self.previous_time=current_time        
        throttle=self.pid_controller.step(vel_err,sample_time)
        brake=0

        if linear_vel==0 and current_vel <0.1:
        	throttle=0
        	brake=400
        elif throttle<0.1 and vel_err<0:
        	throttle=0
        	decel=max(vel_err, self.decel_limit)
        	brake=abs(decel)*self.vehicle_mass*self.wheel_radius

        return throttle, brake, steer
