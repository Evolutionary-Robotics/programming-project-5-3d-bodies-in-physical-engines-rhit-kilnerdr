import pyrosim.pyrosim as ps

l = 1
w = 1
h = .5

x = 0
y = 0
z = 1.25

leg_size = [0.1, 0.1, 1]  # Leg size
leg_offset = 0.25  # How far legs are positioned from the center
leg_height_offset = -0.5  # Height difference for legs relative to the body


def Create_World():
    ps.Start_SDF("box.sdf")
    for i in range(10):
        ps.Send_Cube(name="Box", pos=[x,y,z], size=[1, w, h]) 
        z += h
        l = 0.9 * l
        w = 0.9 * w
        h = 0.9 * h
    ps.End()

def Create_Robot():
    ps.Start_URDF("body.urdf")
    
    # Create the main triangular body
    ps.Send_Cube(name="Body", pos=[x,y,z], size=[1, w, h])   # Body in the center

    # Create the first leg and its joint
    ps.Send_Joint(name="Leg1_Body", parent="Body", child="Foot1", type="revolute", position=[-.20, 0, 1.0])
    ps.Send_Cube(name="Foot1", pos=[-leg_offset, 0, leg_height_offset], size=leg_size)

    # Create the second leg and its joint
    ps.Send_Joint(name="Leg2_Body", parent="Body", child="Foot2", type="revolute", position=[.70, 0, 1.0])
    ps.Send_Cube(name="Foot2", pos=[-leg_offset, 0, leg_height_offset], size=leg_size)

    # Create the third leg and its joint
    ps.Send_Joint(name="Leg3_Body", parent="Body", child="Foot3", type="revolute", position=[.25, .45, 1.0])
    ps.Send_Cube(name="Foot3", pos=[-leg_offset, 0, leg_height_offset], size=leg_size)
    # Create the third leg and its joint
    ps.Send_Joint(name="Leg4_Body", parent="Body", child="Foot4", type="revolute", position=[.25, -.45, 1.0])
    ps.Send_Cube(name="Foot4", pos=[-leg_offset, 0, leg_height_offset], size=leg_size)

    ps.End()

Create_Robot()
# def Create_Robot():
#     ps.Start_URDF("body.urdf")
#     ps.Send_Cube(name="Foot",pos=[x,y,z],size=[l,w,h]) # Parent
#     ps.Send_Joint(name="Foot_Torso", parent="Foot", child="Torso", type="revolute", position = [0.5,0,1.0])
#     ps.Send_Cube(name="Torso",pos=[0.5,0.0,0.5],size=[l,w,h]) # Child
#     ps.End()

# Create_Robot()
