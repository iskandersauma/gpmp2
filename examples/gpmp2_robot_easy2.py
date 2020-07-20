import atexit
import time
from openravepy import *
import orgpmp2.orgpmp2
import time
import types
import prpy.kin
import prpy.rave
import sys
import os
import math



def path_length(traj):
	delta_s = 0 
	N = traj.GetNumWaypoints()
	for i in range(N-1):
		first_pos = traj.GetWaypoint(i)
		second_pos = traj.GetWaypoint(i+1)
		delta_x_2 = math.pow(second_pos[0] - first_pos[0] - 0.9985*(math.sin(second_pos[2]) - math.sin(first_pos[2])),2)
		delta_y_2 = math.pow(second_pos[1] - first_pos[1] - 0.9985*(math.cos(second_pos[2]) - math.cos(first_pos[2])),2)
		delta_z_2 = math.pow(second_pos[3] - first_pos[3],2)
		delta_s += math.sqrt(delta_x_2 + delta_y_2 + delta_z_2)
	return delta_s

def acc_cost(traj):
	tot_acc = 0
	N = traj.GetNumWaypoints()
	for i in range(N-1):
		delta_a  = 0
		first_pos = traj.GetWaypoint(i)
		second_pos = traj.GetWaypoint(i+1)
		delta_a += math.pow((second_pos[5] - first_pos[5])/second_pos[10] ,2)
		delta_a += math.pow((second_pos[6] - first_pos[6])/second_pos[10] ,2)
		delta_a += math.pow((second_pos[7] - first_pos[7])/second_pos[10] ,2)
		delta_a += math.pow((second_pos[8] - first_pos[8])/second_pos[10] ,2)
		delta_a += math.pow((second_pos[9] - first_pos[9])/second_pos[10] ,2)
		tot_acc += math.sqrt(delta_a)
	return tot_acc/N 

def retime_retraj(ortraj, robot, vel_factor=0.5):
  vel_limits = robot.GetDOFVelocityLimits()
  robot.SetDOFVelocityLimits(vel_factor * vel_limits)
  planningutils.RetimeTrajectory(ortraj, hastimestamps=False)
  robot.SetDOFVelocityLimits(vel_limits)
  return ortraj

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/data")
#alt. sys.path.append('data')
import problemsets

# Initialize openrave logging
from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()

# Problemset
problemset = 'robot'

# Start and end joint angles
starts, ends, start, end = problemsets.states(problemset)
start_joints = numpy.array(start[6])
end_joints = numpy.array(end[6])

# Set up openrave and load env
RaveInitialize(True, level=DebugLevel.Info)
atexit.register(RaveDestroy)
e = Environment()
atexit.register(e.Destroy)
e.Load("data/envs/easy2.env.xml")
e.Load('data/robots/robot_model.xml')
e.SetViewer('qtcoin')


# Set up robot
r = e.GetRobots()[0]
raveLogInfo("Robot "+r.GetName()+" (#links: " + str(len(r.GetLinks())) + ") has "+repr(r.GetDOF())+" joints with values:\n"+repr(r.GetDOFValues()))

#import IPython
#IPython.embed()


r.SetTransform([[ 1.0,  0.0,  0.0,  0.0],[ 0.0,  1.0,  0.0,  0.0],[ 0.0,  0.0,  1.0,  0.0],[ 0.0,  0.0,  0.0,  1.0]])
rave_joint_names = [joint.GetName() for joint in r.GetJoints()]
print(rave_joint_names)

print(r.GetActiveDOFValues()) 
print(r.GetDOFValues())


r.SetActiveManipulator('asp_ee')
m = r.GetActiveManipulator()
r.SetActiveDOFs(m.GetArmIndices())
r.SetActiveDOFValues(start_joints)

print(r.GetActiveDOFValues())
print(r.GetActiveDOF())
print(r.GetDOFValues())

#import IPython
#IPython.embed()


r.Grab(e.GetKinBody('obj5'))  # grabs the top object in the pile

# Load gpmp2
m_gpmp2 = RaveCreateModule(e,'orgpmp2')
orgpmp2.orgpmp2.bind(m_gpmp2)

# SDF
# remove right arm links to calculate sdf
l = r.GetLinks()
right_arm_links = ['world', 's_crossbeam', 'n_crossbeam', 'e_cube', 'w_cube', 's_rail', 'n_rail',
  's_slider', 'e_rail', 'w_rail', 'n_slider', 'ne_cube', 'se_cube', 'nw_cube', 'sw_cube', 'carriage',
  'e_carriage_beam', 'w_carriage_beam', 'under_carriage', 'elbow', 'forearm', 'forearm_motor', 
  'forearm_gearbox', 'wrist', 'yaw_gear', 'end_effector']
for i in l:
  if i.GetName() in right_arm_links:
    i.Enable(False)

#import IPython
#IPython.embed()

# Compute distance field for the env and remaining robot
m_gpmp2.computedistancefield(cache_filename='sdf_env_'+problemset+ '_easy2' + '.dat',
  centroid=numpy.array([1.0,1.4,1.0]),extents=numpy.array([2.6 ,2.1 ,1.2]),res=0.01)
# enable robot
r.Enable(True)



# DH parameters of robot start at default
#alpha = numpy.array([1.570796, -1.570796, 1.570796, 3.141593, 3.141593])
#a = numpy.array([0.553500, 0.094000, 0.255000, 0.069000, 0.865000])
#d = numpy.array([1.962100, -0.000000, -0.001000, -0.236000, 0.234000])
#theta = numpy.array([1.570796, -1.570796, -1.570796, -0.000000, 1.570796])

#import IPython
#IPython.embed()

# DH parameters of robot start at testcase pap2
#alpha = numpy.array([1.570796, -1.570796, 1.570796, 3.141593, 3.141593])
#a = numpy.array([0.553500, 0.094000, 0.255000, 0.069000, 0.865000])
#d = numpy.array([1.962100,  1.934000, 0.003000, -0.236000, 1.434000])
#theta = numpy.array([1.570796, -1.570796, -1.570796, -1.183185, 1.570796])

# DH parameters of robot start at testcase easy1
#alpha = numpy.array([1.570796, -1.570796, 1.570796, 3.141593, 3.141593])
#a = numpy.array([0.553500, 0.094000, 0.255000, 0.069000, 0.865000])
#d = numpy.array([1.962100,  1.144000, 0.123000, -0.236000, 1.434000])
#theta = numpy.array([1.570796, -1.570796, -1.570796, 1.000000, 1.570796])

# DH parameters of robot start at testcase easy2
alpha = numpy.array([1.570796, -1.570796, 1.570796, 3.141593, -3.141593])
a = numpy.array([0.553500, 0.094000, 0.255000, 0.069000, -0.865000])
d = numpy.array([1.962100,  1.894000, 0.003000, -0.236000, 1.434000])
theta = numpy.array([1.570796, -1.570796, -1.570796, -1.180000, -1.570796])

# DH parameters of robot for testcase pap1
#alpha = numpy.array([1.570796, -1.570796, 1.570796, 3.141593, 3.141593])
#a = numpy.array([0.553500, 0.094000, 0.255000, 0.069000, 0.865000])
#d = numpy.array([1.962100, 1.044000, 0.003000, -0.236000, 1.434000])
#theta = numpy.array([1.570796, -1.570796, -1.570796, 1.150000, 1.570796])


# DH parameters of robot for testcases 3, 4 and 5
#alpha = numpy.array([1.570796, -1.570796, 1.570796, 3.141593, -3.141593])
#a = numpy.array([0.553500, 0.094000, 0.255000, 0.069000, -0.865000])
#d = numpy.array([1.962100, 1.044000, 0.003000, -0.236000, 1.434000])
#theta = numpy.array([1.570796, -1.570796, -1.570796, -1.400000, -1.570796])

# robot id (full robot in openrave) to link id (just arm for gpmp2) mapping
robot_idx = numpy.array([11, 19, 27, 29])
link_idx = numpy.array([0, 1, 3, 4])

base = numpy.array([1.0, 0.0, 0.0, 0.0, 1.0, 1.4, 1.0])
#base = numpy.array([1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0])

# Run gpmp2 # for (eps - d(x)) use hinge_loss_eps = 0.12
try:
  before = time.time()
  t = m_gpmp2.rungpmp2(
    robot = r,
    end_conf = end_joints,
    dh_a = a,
    dh_alpha = alpha,
    dh_d = d,
    dh_theta = theta,
    robot_idx = robot_idx,
    link_idx = link_idx,
    total_step = 10,
    obs_check_inter = 10,
    output_inter = 10,
    total_time = 5.0,
    fix_pose_sigma = 1e-2,
    fix_vel_sigma = 1e-2,
    cost_sigma = 0.01,
    hinge_loss_eps = 0.02,
    Qc = 1,
    no_report_cost = False,
    no_collision_exception = True)
  after = time.time()
except RuntimeError as ex:
   print(ex)
   t = None 

t = retime_retraj(t,r)
r.SetActiveDOFs(m.GetArmIndices())
r.SetActiveDOFValues(start_joints)
#r.Grab(e.GetKinBody('obj5'))

print "---------------------"
print "Runtime:"
print after - before
print "path length"
print path_length(t)
print "acc cost:"
print acc_cost(t)
print "duration:"
print t.GetDuration()
print "---------------------"

# Run optimized trajectory
try:
  while t is not None:
    raw_input('Press [Enter] to run the trajectory ...\n')
    with e:
      r.GetController().SetPath(t)
except KeyboardInterrupt:
  print

