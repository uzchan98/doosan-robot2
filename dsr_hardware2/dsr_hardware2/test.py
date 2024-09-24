import pinocchio

file = "/home/user/robot.urdf"
print(file)

with open(file, "r") as f:
    content = f.read()

model = pinocchio.buildModelFromXML(content)

# data = model.createData()
