import PyKDL as kdl
import time
import math
import kdl_parser_py.urdf as urdf
import copy

ok, tree = urdf.treeFromFile("ur5.xml")
chain = tree.getChain("base_link", "ee_link")

# chain = kdl.Chain()
# chain.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.Fixed), kdl.Frame(kdl.Vector(0, 0, 0.089159))))
# chain.addSegment(
#     kdl.Segment(
#         name="joint1",
#         joint=kdl.Joint(kdl.Joint.RotZ),
#         f_tip=kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0, 0, 0.089159)),
#         I=kdl.RigidBodyInertia(3.7)))

# chain.addSegment(
#     kdl.Segment(
#         name="joint2",
#         joint=kdl.Joint(kdl.Joint.RotY),
#         f_tip=kdl.Frame(kdl.Rotation.RPY(0, 1.570796325, 0),
#                         kdl.Vector(0, 0.13585, 0)),
#         I=kdl.RigidBodyInertia(8.393)))

# chain.addSegment(
#     kdl.Segment(
#         name="joint3",
#         joint=kdl.Joint(kdl.Joint.RotY),
#         f_tip=kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
#                         kdl.Vector(0, -0.1197, 0.425)),
#         I=kdl.RigidBodyInertia(2.275)))

# chain.addSegment(
#     kdl.Segment(
#         name="joint4",
#         joint=kdl.Joint(kdl.Joint.RotY),
#         f_tip=kdl.Frame(kdl.Rotation.RPY(0, 1.570796325, 0),
#                         kdl.Vector(0, 0, 0.39225)),
#         I=kdl.RigidBodyInertia(1.219)))

# chain.addSegment(
#     kdl.Segment(
#         name="joint5",
#         joint=kdl.Joint(kdl.Joint.RotZ),
#         f_tip=kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0,  0.093, 0)),
#         I=kdl.RigidBodyInertia(1.219)))
# chain.addSegment(
#     kdl.Segment(
#         name="joint6",
#         joint=kdl.Joint(kdl.Joint.RotY),
#         f_tip=kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0, 0, 0.09465)),
#         I=kdl.RigidBodyInertia(0.1879)))


n=6

q=kdl.JntArray(n)
qd_prev=kdl.JntArray(n)
ddq=kdl.JntArray(n)


grav_torques=kdl.JntArray(n)
mass_matrix=kdl.JntSpaceInertiaMatrix(n)
coriolis_vector=kdl.JntArray(n)

grav=kdl.Vector(0, 0, -9.82)

for i in range(n-1):
    qd_prev[i]=0.5
dyn_model=kdl.ChainDynParam(chain, grav)

for i in range(10):
    for i in range(n):
        q[i] += 0.5*0.008
    start_calc=time.time()

    prev_grav = kdl.JntArray( grav_torques)
    prev_mass_matrix =kdl.JntSpaceInertiaMatrix(mass_matrix)
    prev_corriolis_vector = kdl.JntArray(coriolis_vector)

    dyn_model.JntToGravity(q, grav_torques)
    dyn_model.JntToMass(q, mass_matrix)
    dyn_model.JntToCoriolis(q, qd_prev, coriolis_vector)
    duration=time.time() - start_calc

    dif_grav = kdl.JntArray(n)
    dif_mass =kdl.JntSpaceInertiaMatrix(n)
    dif_coriolis =kdl.JntArray(n)
    for i in range(n):
        dif_grav[i] = grav_torques[i]- prev_grav[i]
        dif_coriolis[i] = coriolis_vector[i] - prev_corriolis_vector[i]
        for j in range(n):
            dif_mass[i,j] = mass_matrix[i,j] - prev_mass_matrix[i,j]
    
    # print(f"delta_grav_trq\n", dif_grav, f"\n")
    # print(f"delta_mass_matrix\n", dif_mass, f"\n")
    # print(f"delta_coriolis_vector\n", dif_coriolis, f"\n")
    # print(f"q\n", q, f"\n")
    # print(f"qd\n", qd_prev, f"\n")
    # print(f"calc_duration\n", duration)
    # print(f"----\n\n")
    

    print(f"q")
    for i in range(n):
        print( q[i], ",",  end=' ')
    print(f"\n")

    print(f"qdd")
    for i in range(n):
        print( qd_prev[i], ",", end =' ')
    print(f"\n")

    print(f"delta_coriolis_vector")
    for i in range(n):
        print( "{:.9f}".format(coriolis_vector[i]), ",", end = ' ')
    print(f"\n")

    print(f"delta_grav_trq")
    for i in range(n):
        print( "{:.9f}".format(dif_grav[i]), ",", end = ' ')
    print(f"\n")

    print(f"delta_mass_matrix")
    for i in range(n):
        for j in range(n):
            print( "{:.9f}".format(mass_matrix[i,j]), ",", end = ' ')
        print()
    print()

    print(f"calc_duration\n", duration)
    print(f"----\n\n")


    time.sleep(0.008)
