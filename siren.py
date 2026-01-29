from openscad import *
import math
eps = 0.01
fn=50

toroidal_propeller = osuse("./Custom_Toroidal_Blade.scad")

# All components
add_parameter("height", 10)
add_parameter("slice_count", 50)

# Propeller
add_parameter("blade_count", 3)
add_parameter("blade_length", 85)
add_parameter("blade_width_ratio", 100)
add_parameter("blade_thickness", 2)
add_parameter("blade_attack_angle", 35)
add_parameter("peg_size", 1.5)
add_parameter("lead_width", 3)
add_parameter("lead_count", 3)
add_parameter("hub_thickness", 2)
add_parameter("hub_d", 26)

# Siren intake
add_parameter("slit_count", 9)
add_parameter("center_diameter", 26)
add_parameter("center_hub_diameter", 5)
add_parameter("hub_twist_factor", 100)
add_parameter("wall_thickness", 2)
add_parameter("peg_angle_adjustment", 58)
add_parameter("slit_expansion", 300)

class Main(object):
    
  def __init__(self):
    self.blade_count = blade_count
    self.blade_length = blade_length
    self.blade_width = (blade_length / 100.0) * blade_width_ratio
    self.blade_thickness = blade_thickness
    self.height = height
    self.blade_attack_angle = blade_attack_angle
    self.slit_count = int(slit_count)
    self.slit_expansion = slit_expansion
    self.center_diameter = center_diameter
    self.center_hub_diameter = center_hub_diameter
    self.hub_twist_factor = hub_twist_factor
    self.slice_count = int(slice_count)
    self.hub_d = hub_d
    self.hub_thickness = hub_thickness
    self.lead_width = lead_width
    self.lead_count = int(lead_count)
    self.wall_thickness = wall_thickness
    self.peg_size = peg_size
  
  def peg(self, size):
     return cylinder(d=size, h=size) + [self.lead_width / 2, self.hub_d / 2 - (self.hub_thickness + self.lead_width) / 2, 0]
  
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
    peg_d = self.peg_size
    peg_h = self.peg_size
    peg = self.peg(self.peg_size) + [0, 0, height-eps]
    pegs = [
      peg.rotate([0, 0, 360 / self.lead_count * (i + 0.5)])
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
    return union(leads, propeller, pegs)

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
      scale=[self.slit_expansion / 100, self.slit_expansion / 100],
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
      outer_d=self.center_diameter - 2 * self.wall_thickness + eps) + [0, 0, 1-eps]
    slits = [
      single_slit.rotate([0, 0, i * 360 / self.slit_count])
      for i in range(0, self.slit_count)
    ]
    contents = difference(cylinder(d=self.center_diameter, h=height), slits)
    barrel = difference(
      cylinder(d=self.center_diameter, h=height+1),
      cylinder(d=self.center_diameter-2 * self.wall_thickness, h=self.height+1+2*eps)-[0, 0, eps])
    tip = cylinder(d=self.center_hub_diameter, h=self.height/2+eps) + [0, 0, self.height/2]
    peg = self.peg(self.peg_size + 0.1).rotate([0, 0, peg_angle_adjustment]) - [0, 0, eps]
    pegs = [
      peg.rotate([0, 0, 360 * i / self.lead_count])
      for i in range(0, self.lead_count)
    ]
    return difference(
      union(contents, barrel),
      tip,
      pegs)
  
  def gen(self):
    return union(
      self.propeller(), 
      self.intake_slits() + [100, 100, 0]
    )

main = Main()
show(main.gen())
