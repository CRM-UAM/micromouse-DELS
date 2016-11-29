// Maze solving robot based on ESP-Multi-Board
// By Víctor Uceda & Carlos García
//   Club de Robótica-Mecatrónica,
//   Universidad Autónoma de Madrid
//   http://crm-uam.github.io
// License: CC-BY-SA 4.0 (Attribution-ShareAlike 4.0 International, http://creativecommons.org/licenses/by-sa/4.0/)
// Designed with http://www.openscad.org/

// Increase the resolution of default shapes
$fa = 5; // Minimum angle for fragments [degrees]
$fs = 0.5; // Minimum fragment size [mm]

module ESP_multi_board(holes=false) {
    if(holes) {
        translate([0,0,-1]) linear_extrude(height=1) hull() translate([-134.74,110,0]) import("ESP_multi_board-Edge.Cuts.dxf");
        // Screws
        translate([12.47+0.6,2.53,0]) cylinder(r=2/2,h=10,center=true);
        translate([-12.47+0.6,2.53,0]) cylinder(r=2/2,h=10,center=true);
        // Space for solder pads
        translate([-32.3,0.6,-2]) cube([17,3,2]);
        translate([17,0.6,-2]) cube([18.3,3,2]);
        translate([17+18.3-3,0.6,-2]) cube([3,13,2]);
    } else translate([-134.74,110,-0.5]) {
        import("ESP_multi_board-Edge.Cuts.dxf");
        import("ESP_multi_board-F.Cu.dxf");
    }
    // ESP module
    translate([-16/2+0.6,-7,0]) cube([16,24,1]);
}

module IR_emmiter(holes=false) { // SFH4545
    if(holes) {
        cylinder(r=5.65/2+0.2,h=1+0.2);
        translate([0,0,-19.9]) cylinder(r=5.65/2+0.2,h=20); //hole to introduce component
        cylinder(r1=5.5/2+0.2,r2=4.4/2+0.2,h=4.5);
        translate([0,0,4.5]) scale([1,1,1.25]) sphere(r=4.4/2+0.2);
        translate([2.54/2,0,-15/2]) cube([0.6,0.6,15],center=true);
        translate([-2.54/2,0,-15/2]) cube([0.6,0.6,15],center=true);
    } else {
        cylinder(r=5.65/2,h=1);
        cylinder(r1=5.5/2,r2=4.4/2,h=4.5);
        translate([0,0,4.5]) scale([1,1,1.25]) sphere(r=4.4/2);
        translate([2.54/2,0,-15/2]) cube([0.5,0.5,15],center=true);
        translate([-2.54/2,0,-15/2]) cube([0.5,0.5,15],center=true);
    }
}

module IR_receiver(holes=false) { // TEFT4300
    if(holes) {
        cylinder(r=3.94/2+0.2,h=1.25);
        translate([0,0,-19.9]) cylinder(r=3.94/2+0.2,h=20); //hole to introduce component
        cylinder(r1=3/2+0.2,r2=2.83/2+0.2,h=3.45);
        translate([0,0,3.45]) sphere(r=2.83/2+0.2);
        cube([3+0.2,0.43+0.2,2*1.3],center=true);
        translate([2.54/2,0,-15/2]) cube([0.43+0.2,0.43+0.2,15],center=true);
        translate([-2.54/2,0,-15/2]) cube([0.43+0.2,0.43+0.2,15],center=true);
    } else {
        cylinder(r=3.94/2,h=1.25);
        cylinder(r1=3/2,r2=2.83/2,h=3.45);
        translate([0,0,3.45]) sphere(r=2.83/2);
        cube([3,0.43,2*1.3],center=true);
        translate([2.54/2,0,-15/2]) cube([0.43,0.43,15],center=true);
        translate([-2.54/2,0,-15/2]) cube([0.43,0.43,15],center=true);
    }
}

module IR_emmiter_receiver(holes=false) {
    translate([0,7,0]) {
        translate([0,6,-2.38]) IR_emmiter(holes);
        IR_receiver(holes);
    }
}

module IR_sensor() {
    rotate([90,0,0]) {
        difference() {
            translate([-9/2,0,-5.5]) hull() {
                cube([9,0.1,8]);
                translate([9/2,13,0]) cylinder(r=9/2,h=8);
            }
            IR_emmiter_receiver(holes=true);
        }
        %IR_emmiter_receiver();
    }
}

module sensor_holder() {
    difference() {
        translate([0,25,-3]) cylinder(r=83/2, h=3);
        translate([0,0,0.01]) ESP_multi_board(holes=true);
        translate([-50,6,-50]) cube([100,100,100]);
    }
    translate([0,-14,0]) IR_sensor();
    translate([-20,-5,0]) rotate([0,0,-90]) IR_sensor();
    translate([20,-5,0]) rotate([0,0,90]) IR_sensor();
}



%ESP_multi_board();
difference() {
    sensor_holder();
    translate([0,-12,0]) cylinder(r=4/2, h=8, center=true);
}