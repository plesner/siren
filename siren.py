from openscad import *
import math
eps = 0.01

toroidal_propeller = osuse("./toroidal_propeller.scad")

add_parameter("blade_count", 3)
add_parameter("blade_length", 100)
add_parameter("blade_width", 100)
add_parameter("height", 10)
add_parameter("blade_attack_angle", 35)
add_parameter("slit_count", 8)

def propeller(blades, blade_length, blade_width, height, hub_d, blade_thickness, hub_thickness):
  lead_width = 3
  lead_count = 3
  lead_offset = hub_d / 2 - hub_thickness - lead_width + eps
  lead = linear_extrude(
    square(lead_width) + [0, lead_offset], 
    height=height,
    slices=50,
    twist=180 / lead_count)
  leads = [
    lead.rotate([0, 0, 360 / lead_count * i]) 
    for i in range(0, lead_count)
  ]
  propeller = toroidal_propeller.toroidal_propeller(
    blades = blades,
    height = height,
    blade_length = blade_length,
    blade_width = blade_width,
    blade_thickness = blade_thickness,
    blade_hole_offset = 1.4,
    blade_attack_angle = blade_attack_angle,
    blade_offset = -6,
    hub_height = height,
    hub_d = hub_d,
    hub_screw_d = hub_d - 2 * hub_thickness)
  return union(leads, propeller)
  

def intake_slit_2d(inner_d, outer_d, angle):
  inner = circle(d=inner_d)
  outer = circle(d=outer_d, angle=angle)
  return rotate(difference(outer, inner), [0, 0, -angle / 2])

def intake_slit(height, inner_d, outer_d, count):
  raw_profile = intake_slit_2d(inner_d, outer_d, 180 / count)
  profile = raw_profile - [inner_d, 0, 0] 
  top = linear_extrude(
    profile,
    height=height,
    scale=[3, 3],
    slices=50,
    twist=360 / count,
    convexity=1) + [inner_d, 0, 0]
  bottom = linear_extrude(
    profile,
    height=height+eps) + [inner_d, 0, -height]
  return union(top, bottom)

def intake_slits(height, inner_d, outer_d, count, wall_thickness):
  single_slit = intake_slit(
    height=height + 2 * eps,
    inner_d=inner_d,
    outer_d=outer_d + eps,
    count=count) + [0, 0, 1-eps]
  slits = [
    single_slit.rotate([0, 0, i * 360 / count])
    for i in range(0, count)
  ]
  contents = difference(cylinder(d=outer_d, h=height), slits)
  barrel = difference(
    cylinder(d=outer_d, h=height),
    cylinder(d=outer_d-wall_thickness, h=height+2*eps)-[0, 0, eps])
  tip = cylinder(d=inner_d, h=height/2+eps) + [0, 0, height/2]
  return difference(union(contents, barrel), tip)

show(intake_slits(
  height=10,
  inner_d=10,
  outer_d=40,
  count=int(slit_count),
  wall_thickness=2
))

show(propeller(
  blades=blade_count,
  blade_length=blade_length,
  blade_width=blade_width,
  height=height,
  hub_d=26,
  blade_thickness=3,
  hub_thickness=2) + [100, 100, 0])
