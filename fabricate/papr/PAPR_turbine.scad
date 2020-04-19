/*
difference(){import("PAPR_turbine.stl", convexity=3);
  translate([0, 0, -4.25]) rotate([180, 0, 0])
    import("m6_screw.stl", convexity=3);
}
*/


$fn=50;
R = 36.5;
BladeR = 30;
h=2;
H = 9;

module arc(radius, angle, thickness){
  angles = [0, angle];
  points = [
	    for(a = [angles[0]:1:angles[1]]) [radius * cos(a), radius * sin(a)]
	    ];
  linear_extrude(height=thickness)polygon(concat([[0, 0]], points));
}

module blade(){
  intersection(){
    translate([0, 15, 0])rotate([0, 0, 20])translate([-BladeR, 0, 0])
      difference(){
      angle = 50;
      arc(BladeR, angle, H+2);
      rotate([0, 0, -1])translate([0, 0, -1])arc(BladeR - 1, angle + 2, H + 4);
    }
    cylinder(r=R, h=H+2);
  }
}

module fan(){
  difference(){
    union(){
      color("red")
	for(i=[0: 8]){
	  rotate([0, 0, i*360/9])blade();
	}
      translate([0, 0, 0])
	color("blue"){
	translate([0, 0, -2])cylinder(r=R, h=h, $fn=100);
	translate([0, 0, H])difference(){
	  cylinder(r=R, h=h, $fn=100);
	  translate([0, 0, -1])cylinder(r=R-3, h=h+2, $fn=100);
	}
      }
      color("green")
	translate([0, 0, H]){
	difference(){
	  cylinder(r1=18, r2=17, h=3);
	  translate([0, 0, -1])cylinder(r=15, h=5);
	}
      }

      difference(){
	cylinder(d=9, h=6);
      }
    }

    translate([0, 0, -6.25]) rotate([180, 0, 0])
      import("m6_screw.stl", convexity=3);
  }
}

//main body
dia = 20;
difference(){
  union(){
    cylinder(r=R + 4, h = H + 12);
    translate([0, R + 3 - dia/2, dia/2+1])rotate([0, 90, 0])cylinder(d=dia, h=70);

    for(i=[0:7]){
      rotate([0, -90, i*45+22.5])translate([2, R+4, -1])linear_extrude(height=2)polygon(points=[[0,0],[0,15],[19,0]] );
    }
  }
  translate([0, 0, -1])cylinder(r=R + 2, h = H + 12);
  translate([0, 0, -1])cylinder(r=18, h = H + 100);
  translate([0, R + 3 - dia/2, dia/2+1])rotate([0, 90, 0])translate([0, 0, -1])cylinder(d=dia-2, h=72);
}

// flange
n_hole = 8;
translate([0, 0, 0])difference(){
  rotate([0, 0, 22.5])cylinder(r=R + 20, h=2, $fn=8);
  translate([0, 0, -1])cylinder(r=R + 2, h=4);
  for(i=[0:n_hole-1]){
    rotate([0, 0, i*360/n_hole+0])translate([R + 10, 0, -1])cylinder(d=4, h=10);
  }
}
translate([0, 0, 4])fan();
/*

*/
