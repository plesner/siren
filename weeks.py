from openscad import *
import math

fn = 200

outer_radius = 24
wall_width = 1
outer_ring_inner_radius = outer_radius - 8
outer_ring_height = 4.6
outer_fixpoint_count = 13
inner_fixpoint_count = 12

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

def twist(outer_diameter, width, height):
  wide_angle = 1.0 / inner_fixpoint_count
  narrow_angle = 1.0 / outer_fixpoint_count
  outer_d = outer_diameter
  inner_d = outer_diameter - width
  narrow_offset = narrow_angle / 3
  wide_offset = -1.0 / (inner_fixpoint_count * outer_fixpoint_count)
  marker = 0.0
  points = [
    [0, inner_d],
    [1 * narrow_angle - narrow_offset, inner_d],
    [1 * narrow_angle + narrow_offset, inner_d],
    [2 * narrow_angle - narrow_offset, inner_d],
    [2 * narrow_angle + narrow_offset, inner_d],
    [3 * narrow_angle - narrow_offset, inner_d],
    [3 * narrow_angle + wide_offset, outer_d],
    [4 * narrow_angle - wide_offset, outer_d],
    [4 * narrow_angle + narrow_offset, inner_d],
    [5 * narrow_angle - narrow_offset, inner_d],
    [5 * narrow_angle + narrow_offset, inner_d],
    [6 * narrow_angle - narrow_offset, inner_d],
    [6 * narrow_angle + narrow_offset, inner_d],
    [7 * narrow_angle - narrow_offset, inner_d],
    [7 * narrow_angle + wide_offset, outer_d],
    [8 * narrow_angle - wide_offset, outer_d],
    [8 * narrow_angle + narrow_offset, inner_d],
    [9 * narrow_angle - narrow_offset, inner_d],
    [9 * narrow_angle + narrow_offset, inner_d],
    [10 * narrow_angle - narrow_offset, inner_d],
    [10 * narrow_angle + narrow_offset, inner_d],
    [11 * narrow_angle - narrow_offset, inner_d],
    [11 * narrow_angle + wide_offset, outer_d],
    [12 * narrow_angle - wide_offset, outer_d],
    [12 * narrow_angle + narrow_offset, inner_d],
    [1, inner_d]
  ]
  def xsection(a):
    before = None
    after = None
    for pair in points:
      if pair[0] <= a:
        before = pair
      elif pair[0] >= a:
        after = pair
        break
    w = after[0] - before[0]
    t = (after[0] - a) / w
    radius = before[1] * t + after[1] * (1 - t)
    return [
      [radius, 0],
      [radius, height],
      [radius - width, height],
      [radius - width, 0]
    ]
  return rotate_extrude(xsection, fn=300)

grooves = twist(outer_radius - 3, 2.0, 3) + [0, 0, 1]

root = union(
  difference(outer_ring_bottom, grooves, dimples),
  outer_ring_outer_wall,
  fixpoints)

show(root)
