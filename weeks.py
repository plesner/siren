from openscad import *
import math

fn = 200
tol = 0.1
eps = 0.01

outer_radius = 24
outer_width = 7
wall_width = 1
outer_ring_inner_radius = outer_radius - outer_width
outer_ring_height = 4.6
outer_fixpoint_count = 13
inner_fixpoint_count = 12
outer_ring_bottom_height = 2.8
fixpoint_height = 0.2

bottom_outer_radius = outer_radius - wall_width - tol
bottom_inner_radius = 6.5
bottom_height = 2.6
bottom_wall_height = 1.8
bottom_wall_width = 6.2

positions = set([2, 7, 11])


def open_cylinder(outer_radius, inner_radius, height):
  outer = cylinder(r=outer_radius, h=height)
  inner = cylinder(r=inner_radius, h=height + 2 * eps)
  return outer - (inner - [0, 0, eps])

def fixpoint_strip(outer_radius, inner_radius, height, count):
  shrinkage_factor = 0.05
  shrinkage = shrinkage_factor * (180 / count)
  fixpoint = linear_extrude(
    difference(
      circle(r=outer_radius, angle=(180 / count) - shrinkage), 
      circle(r=inner_radius)),
    height=height)
  return union([
    fixpoint.rotate([0, 0, (i + 0.25) * 360 / count + (shrinkage / 2)])
      for i in range(0, count)])

def extrude_waypoints(inner_d, outer_d):
  wide_angle = 1.0 / inner_fixpoint_count
  narrow_angle = 1.0 / outer_fixpoint_count
  narrow_offset = narrow_angle / 2
  wide_offset = -1.0 / (inner_fixpoint_count * outer_fixpoint_count)
  marker = 0.0
  point_groups = [[[0, inner_d]]]
  for i in range(1, 13):
    if i in positions:
      offset = wide_offset
      d = outer_d
    else:
      offset = narrow_offset
      d = inner_d
    point_groups.append([[i * narrow_angle + offset, d]])
    point_groups.append([[(i + 1) * narrow_angle - offset, d]])
  point_groups.append([[1, inner_d]])
  return [p for ps in point_groups for p in ps]

def extrude_interpolate(points, width, height):
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
  return rotate_extrude(
    xsection,
    fn=200)

def outer_ring():
  bottom = open_cylinder(
    outer_radius=outer_radius,
    inner_radius=outer_ring_inner_radius,
    height=outer_ring_bottom_height)

  outer_wall = open_cylinder(
    outer_radius=outer_radius,
    inner_radius=outer_radius - wall_width,
    height=outer_ring_height)
  
  fixpoints = translate(
    fixpoint_strip(
      outer_radius=outer_radius - wall_width,
      inner_radius=outer_radius - 2 * wall_width,
      height=fixpoint_height + eps,
      count=outer_fixpoint_count),
    [0, 0, outer_ring_bottom_height - eps])

  dimple = translate(
    cylinder(d1=2, d2=0, h=1),
    [outer_radius - outer_width / 2, 0, -eps])

  dimples = union([
    dimple.rotate([0, 0, i * 360 / outer_fixpoint_count])
      for i in range(0, outer_fixpoint_count)])

  def extrude_groove(outer_radius, width, height):
    points = extrude_waypoints(
      inner_d = outer_radius - width,
      outer_d = outer_radius)
    return rotate(
      extrude_interpolate(
        points=points,
        width=width,
        height=height),
      [0, 0, 180.0 / outer_fixpoint_count])

  grooves = extrude_groove(
    outer_radius=outer_radius - 2,
    width=2.0,
    height=1 + eps) + [0, 0, 2]

  return union(
    difference(bottom, grooves, dimples),
    outer_wall,
    fixpoints)

def bottom():
  bottom = open_cylinder(
    outer_radius=bottom_outer_radius,
    inner_radius=bottom_inner_radius,
    height=bottom_height - bottom_wall_height)

  outer_wall = open_cylinder(
    outer_radius=bottom_outer_radius,
    inner_radius=bottom_outer_radius - bottom_wall_width,
    height=bottom_height)
  
  def extrude_groove(outer_radius, width, height):
    points = extrude_waypoints(
      outer_d = bottom_outer_radius - wall_width - 2,
      inner_d = bottom_outer_radius - wall_width)
    return rotate(
      extrude_interpolate(
        points=points,
        width=width,
        height=height),
      [0, 0, 180.0 / outer_fixpoint_count])

  grooves = extrude_groove(
    outer_radius=outer_radius - 2,
    width=2.0,
    height=1 + eps) + [0, 0, 2]

  fixpoints = translate(
    fixpoint_strip(
      outer_radius=outer_radius - wall_width - tol,
      inner_radius=outer_radius - 2 * wall_width,
      height=fixpoint_height + eps,
      count=outer_fixpoint_count),
    [0, 0, bottom_height - eps])
  
  return difference(union(bottom, outer_wall, fixpoints), grooves)


show(translate(outer_ring(), [50, 50, 0]))
show(bottom())
