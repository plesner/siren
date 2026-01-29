from openscad import *
import math
eps = 0.01

toroidal_propeller = osuse("./Custom_Toroidal_Blade.scad")

add_parameter("blade_count", 3)
add_parameter("blade_length", 100)
add_parameter("blade_width", 100)
add_parameter("blade_thickness", 2)
add_parameter("height", 10)
add_parameter("blade_attack_angle", 35)
add_parameter("slit_count", 8)
add_parameter("center_diameter", 40)
add_parameter("center_hub_diameter", 10)
add_parameter("hub_twist_factor", 100)
add_parameter("slice_count", 50)

class Main(object):
    
  def __init__(self):
    self.blade_count = blade_count
    self.blade_length = blade_length
    self.blade_width = blade_width
    self.blade_thickness = blade_thickness
    self.height = height
    self.blade_attack_angle = blade_attack_angle
    self.slit_count = int(slit_count)
    self.center_diameter = center_diameter
    self.center_hub_diameter = center_hub_diameter
    self.hub_twist_factor = hub_twist_factor
    self.slice_count = int(slice_count)
    self.hub_d = 26
    self.hub_thickness = 2
    self.lead_width = 3
    self.lead_count = 3
    self.wall_thickness = 2
  
  def propeller(self):
    lead_offset = self.hub_d / 2 - self.hub_thickness - self.lead_width + eps
    lead = linear_extrude(
      square(self.lead_width) + [0, lead_offset], 
      height=height,
      slices=int(self.slice_count),
      twist=180 / self.lead_count)
    leads = [
      lead.rotate([0, 0, 360 / self.lead_count * i]) 
      for i in range(0, self.lead_count)
    ]
    propeller = toroidal_propeller.toroidal_propeller(
      blades = self.blade_count,
      height = self.height,
      blade_length = self.blade_length,
      blade_width = self.blade_width,
      blade_thickness = self.blade_thickness,
      blade_attack_angle = self.blade_attack_angle,
      hub_height = self.height,
      hub_d = self.hub_d,
      hub_screw_d = self.hub_d - 2 * self.hub_thickness)
    return union(leads, propeller)

  def intake_slit_2d(self, inner_d, outer_d, angle):
    inner = circle(d=inner_d)
    outer = circle(d=outer_d, angle=angle)
    return rotate(difference(outer, inner), [0, 0, -angle / 2])

  def intake_slit(self, height, inner_d, outer_d):
    raw_profile = self.intake_slit_2d(inner_d, outer_d, 180 / self.slit_count)
    profile = raw_profile - [inner_d, 0, 0] 
    top = linear_extrude(
      profile,
      height=height,
      scale=[3, 3],
      slices=self.slice_count,
      twist=self.hub_twist_factor * 3.6 / self.slit_count,
      convexity=1) + [inner_d, 0, 0]
    bottom = linear_extrude(
      profile,
      height=height+eps) + [inner_d, 0, -height]
    return union(top, bottom)

  def intake_slits(self):
    single_slit = self.intake_slit(
      height=self.height + 2 * eps,
      inner_d=self.center_hub_diameter,
      outer_d=self.center_diameter - self.wall_thickness + eps) + [0, 0, 1-eps]
    slits = [
      single_slit.rotate([0, 0, i * 360 / self.slit_count])
      for i in range(0, self.slit_count)
    ]
    contents = difference(cylinder(d=self.center_diameter, h=height), slits)
    barrel = difference(
      cylinder(d=self.center_diameter, h=height+1),
      cylinder(d=self.center_diameter-self.wall_thickness, h=self.height+1+2*eps)-[0, 0, eps])
    tip = cylinder(d=self.center_hub_diameter, h=self.height/2+eps) + [0, 0, self.height/2]
    return difference(union(contents, barrel), tip)
  
  def gen(self):
    return union(self.propeller(), self.intake_slits() + [100, 100, 0])

main = Main()
show(main.gen())
