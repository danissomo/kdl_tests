import PyKDL as kdl
import time
import math
import kdl_parser_py.urdf as urdf
import copy

ok, tree = urdf.treeFromFile("ur5.xml")
chain = tree.getChain("base_link", "ee_link")

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
    q[i] = 0.5
dyn_model=kdl.ChainDynParam(chain, grav)

dyn_model.JntToGravity(q, grav_torques)
dyn_model.JntToMass(q, mass_matrix)
dyn_model.JntToCoriolis(q, qd_prev, coriolis_vector)

print(f"mass_matrix_1")
for i in range(n):
    for j in range(n):
        print( "{:.9f}".format(mass_matrix[i,j]), ",", end = ' ')
    print()
print()

for i in range(n-1):
    qd_prev[i]=0.5
    q[i] += qd_prev[i]*0.004+0.06
start_calc=time.time()

prev_grav = kdl.JntArray( grav_torques)
prev_mass_matrix =kdl.JntSpaceInertiaMatrix(mass_matrix)
prev_corriolis_vector = kdl.JntArray(coriolis_vector)


dyn_model.JntToGravity(q, grav_torques)
dyn_model.JntToMass(q, mass_matrix)
dyn_model.JntToCoriolis(q, qd_prev, coriolis_vector)
duration=time.time() - start_calc

print(f"mass_matrix_2")
for i in range(n):
    for j in range(n):
        print( "{:.9f}".format(mass_matrix[i,j]), ",", end = ' ')
    print()
print()
dif_grav = kdl.JntArray(n)
dif_mass =kdl.JntSpaceInertiaMatrix(n)
dif_coriolis =kdl.JntArray(n)
for i in range(n):
    dif_grav[i] = grav_torques[i]- prev_grav[i]
    dif_coriolis[i] = coriolis_vector[i] - prev_corriolis_vector[i]
    for j in range(n):
        dif_mass[i,j] = mass_matrix[i,j] - prev_mass_matrix[i,j]




print(f"q")
for i in range(n):
    print( q[i], ",",  end=' ')
print(f"\n")

print(f"qd")
for i in range(n):
    print( qd_prev[i], ",", end =' ')
print(f"\n")

print(f"delta_coriolis_vector")
for i in range(n):
    print( "{:.9f}".format(dif_coriolis[i]), ",", end = ' ')
print(f"\n")

print(f"delta_grav_trq")
for i in range(n):
    print( "{:.9f}".format(dif_grav[i]), ",", end = ' ')
print(f"\n")

print(f"delta_mass_matrix")
for i in range(n):
    for j in range(n):
        print( "{:.9f}".format(dif_mass[i,j]), ",", end = ' ')
    print()
print()

print(f"calc_duration\n", duration)
print(f"----\n\n")


    #time.sleep(0.008)
