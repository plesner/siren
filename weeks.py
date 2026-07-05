from openscad import *

fn = 100

outer_radius = 24
wall_width = 1
outer_ring_inner_radius = outer_radius - 8
outer_ring_height = 4.6
outer_fixpoint_count = 13

eps = 0.01

def open_cylinder(outer_radius, inner_radius, height):
  outer = cylinder(r=outer_radius, h=height)
  inner = cylinder(r=inner_radius, h=height + 2 * eps)
  return outer - (inner - [0, 0, eps])

outer_ring_bottom = open_cylinder(
  outer_radius=outer_radius,
  inner_radius=outer_ring_inner_radius,
  height=2.8)

outer_ring_outer_wall = open_cylinder(
  outer_radius=outer_radius,
  inner_radius=outer_radius - wall_width,
  height=outer_ring_height)
  
fixpoint = linear_extrude(
  difference(
    circle(
      r=outer_radius - wall_width,
      angle=180 / outer_fixpoint_count), 
    circle(
      r=outer_radius - 2 * wall_width)),
  height=3)

fixpoints = union([
  fixpoint.rotate([0, 0, (i + 0.25) * 360 / outer_fixpoint_count])
    for i in range(0, outer_fixpoint_count)])

dimple = cylinder(d1=2, d2=0, h=1) + [outer_radius - 4, 0, -eps]

dimples = union([
  dimple.rotate([0, 0, i * 360 / outer_fixpoint_count])
    for i in range(0, outer_fixpoint_count)])

engaged_outer_groove = linear_extrude(
  difference(
    rotate(
      circle(
        r=outer_radius - 3,
        angle=360 / outer_fixpoint_count + 2 * eps),
        [-eps]),
    circle(
      r=outer_radius - 5)),
  height=3) + [0, 0, 1]

locked_outer_groove = linear_extrude(
  difference(
    rotate(
      circle(
        r=outer_radius - 5,
        angle=360 / outer_fixpoint_count + 2 * eps),
      [-eps]), 
    circle(
      r=outer_radius - 7)),
  height=3) + [0, 0, 1]

groove_pattern = [
  locked_outer_groove,
  engaged_outer_groove,
  locked_outer_groove,
  locked_outer_groove,
  locked_outer_groove,
  engaged_outer_groove,
  locked_outer_groove,
  locked_outer_groove,
  locked_outer_groove,
  locked_outer_groove,
  engaged_outer_groove,
  locked_outer_groove,
  locked_outer_groove
]

grooves = union([
  groove_pattern[i].rotate([0, 0, i * 360 / outer_fixpoint_count])
    for i in range(0, outer_fixpoint_count)])

root = union(
  difference(outer_ring_bottom, grooves, dimples),
  outer_ring_outer_wall,
  fixpoints)

show(root)
