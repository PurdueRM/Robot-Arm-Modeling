# import xml.etree.ElementTree as ET
# import mujoco
# import os

# with open("arm/urdf/arm2.SLDASM.urdf", "r") as f:
#     xml_text = f.read()

# tree = ET.ElementTree(ET.fromstring(xml_text))
# root = tree.getroot()
# mesh_dir = os.path.abspath("./arm/meshes")

# for mesh in root.iter("mesh"):
#     filename = mesh.attrib.get("filename", "")
#     if filename.startswith("../"):
#         mesh.attrib["filename"] = "./arm/" + filename[len("../"):]
#     # filename = mesh.attrib.get("filename", "")
#     # base = os.path.basename(filename)
#     # abs_path = os.path.join(mesh_dir, base)
#     # mesh.attrib["filename"] = abs_path
#     # if not os.path.exists(abs_path):
#     #     print("WARNING: mesh not found:", abs_path)

# fixed_xml = ET.tostring(root, encoding="unicode")
# with open("robot.xml", "w") as f:
#     f.write(fixed_xml)

# # model = mujoco.MjModel.from_xml_string(fixed_xml)
# model = mujoco.MjModel.from_xml_path("robot.xml")
# data = mujoco.MjData(model)

# import os
# path = "./arm/meshes/base_link.STL"
# print("Exists?", os.path.exists(path))

# import mujoco
# model = mujoco.MjModel.from_xml_path("arm.urdf")
# mujoco.mj_saveLastXML("arm.xml", model)

import mujoco
xml = """
<mujoco>
  <compiler meshdir="arm/meshes" />
  <include file="arm.urdf"/>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)