eps=1/128;
//smoothness of the model
facets = 100;
$fn = facets;

module toroidal_propeller(
  blades = 4,
  height = 12,
  blade_length = 60,
  blade_width = 60,              
  blade_thickness = 1,
  blade_hole_offset = 0,        
  blade_attack_angle = 40,
  blade_offset = 0,              
  blade_safe_direction = "PREV", // [PREV,NEXT]
  hub_height = 12,                 
  hub_d = 22,                     
  hub_screw_d = 21,              
  hub_notch_height = 10,           
  hub_notch_d = 0                 
){
    l = height / tan(blade_attack_angle);
    p = 2 * PI * blade_length/2;

    difference(){
        union(){
            linear_extrude(height=height, twist=l/p  * 360, convexity=2){
                blades2D(
                    n = blades,
                    height = height,
                    length = blade_length,
                    width = blade_width,
                    thickness = blade_thickness,
                    hole_offset = blade_hole_offset,
                    blade_direction = blade_attack_angle > 0 ? 1 : -1,
                    offset = blade_offset,
                    blade_safe_direction = 
                        blade_safe_direction == "PREV"? 1 : 
                        blade_safe_direction == "NEXT"? -1: 
                        0 // default
                );
            }

            cylinder(d = hub_d, h = hub_height);
        }
        translate([0,0,-eps]){
            cylinder(d = hub_screw_d, h = hub_height + 2*eps);
            cylinder(d = hub_notch_d, h = hub_notch_height + eps);
        }
    }
}

module blades2D(n, height, length, width, thickness, hole_offset, blade_direction, offset, blade_safe_direction){
    for(a=[0:n-1]){
        difference(){
            rotate([0,0,a*(360/n)]){
                translate([offset,0,0]) blade2D(
                    height = height,        // height
                    length = length,        // blade length in mm
                    width = width,          // blade width in mm
                    thickness = thickness,  // blade thickness in mm
                    hole_offset = hole_offset    // blade hole offset
                );
            }

            cw_ccw_mult = blade_direction * blade_safe_direction;
            rotate([0,0, (a + cw_ccw_mult) * (360/n)])
                translate([length/2 + hole_offset + offset,0,0])
                    scale([1, (width-thickness)/(length-thickness)]) circle(d=length-thickness);

        }
    }
}


module blade2D (height, length, width, thickness, hole_offset){
    difference(){
        translate([length/2,0,0])
            scale([1, width/length]) circle(d=length);

        translate([length/2 + hole_offset,0,0])
            scale([1, (width-thickness)/(length-thickness)]) circle(d=length-thickness);
    }
}
