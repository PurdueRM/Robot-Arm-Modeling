import mujoco
model = mujoco.MjModel.from_xml_path('arm/urdf/arm.urdf')
mujoco.mj_saveLastXML('arm.xml', model)
