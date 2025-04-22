import bpy
import os
from math import sqrt

# To run this file use this in terminal
# blender --background --python dae_conv.py

# Clear existing objects
bpy.ops.wm.read_factory_settings(use_empty=True)

# Settings
marker_path = "marker_4x4_0.png"  # Change this
output_dae = "marker_4x4_0.dae"  # Change this
marker_size = 0.1  # Base size in meters (will be applied to the longer dimension)

# Load image to get dimensions
image = bpy.data.images.load(marker_path)
width, height = image.size
aspect_ratio = width / height

# Create plane with correct proportions
if width > height:
    plane_width = marker_size
    plane_height = marker_size / aspect_ratio
else:
    plane_height = marker_size
    plane_width = marker_size * aspect_ratio

bpy.ops.mesh.primitive_plane_add(size=1.0)
plane = bpy.context.object
plane.scale.x = plane_width
plane.scale.y = plane_height

# Add material with texture
mat = bpy.data.materials.new(name="MarkerMaterial")
mat.use_nodes = True
bsdf = mat.node_tree.nodes["Principled BSDF"]
tex_image = mat.node_tree.nodes.new('ShaderNodeTexImage')
tex_image.image = image
mat.node_tree.links.new(bsdf.inputs['Base Color'], tex_image.outputs['Color'])

# Assign material to plane
if plane.data.materials:
    plane.data.materials[0] = mat
else:
    plane.data.materials.append(mat)

# Export as DAE
bpy.ops.wm.collada_export(filepath=output_dae)