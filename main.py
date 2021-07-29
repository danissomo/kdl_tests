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
qdd=kdl.JntArray(n)


grav_torques=kdl.JntArray(n)
mass_matrix=kdl.JntSpaceInertiaMatrix(n)
coriolis_vector=kdl.JntArray(n)

grav=kdl.Vector(0, 0, -9.82)

for i in range(n-1):
    qd_prev[i]=0.5
    q[i] = 0.5

start_calc=time.time()
dyn_model=kdl.ChainDynParam(chain, grav)


print(f"mass_matrix_1")
for i in range(n):
    for j in range(n):
        print( "{:.9f}".format(mass_matrix[i,j]), ",", end = ' ')
    print()
print()

for i in range(n-1):
    qd_prev[i]+= 0.024
    qdd[i] = 0.024/0.008
    q[i] += qd_prev[i]*0.008 +  qdd[i]*0.008*0.008/2


dyn_model.JntToMass(q, mass_matrix)

print(f"mass_matrix_1")
for i in range(n):
    for j in range(n):
        print( "{:.9f}".format(mass_matrix[i,j]), ",", end = ' ')
    print()
print()
result_vec =kdl.JntArray(n)
kdl.Multiply(mass_matrix, qdd, result_vec)

print(f"tau_a")
for i in range(n):
        print( "{:.9f}".format(result_vec[i]), ",", end = ' ')
print()

duration = time.time() - start_calc

print(f"q")
for i in range(n):
    print( q[i], ",",  end=' ')
print(f"\n")



print(f"calc_duration\n", duration)
print(f"----\n\n")


    #time.sleep(0.008)
