// Reference: https://cdn-learn.adafruit.com/assets/assets/000/122/460/original/sensors_SG_10x5_fab_print.png

$fa = .1;
$fs = 0.04;

BOARD_DEPTH = 2;

punch_depth = .1;
h = -1 * punch_depth / 2;

in2mm = 25.4;
p1in = .1*in2mm;
p15in = .15*in2mm;
p2in = .2*in2mm;
p3in = .3*in2mm;
p6in = .6*in2mm;

difference() {
    union() {
        cube([1 * in2mm, .2 * in2mm, BOARD_DEPTH]);
        cube([.2 * in2mm, 6.2 * in2mm, BOARD_DEPTH]);
        translate([0, 1.8 * in2mm, 0])
            cube([1 * in2mm, .2 * in2mm, BOARD_DEPTH]);
        translate([.8*in2mm, 2 * in2mm, 0])
            cube([.2 * in2mm, 1.8 * in2mm, BOARD_DEPTH]);
        translate([0, 4.6 * in2mm, 0])
            cube([1 * in2mm, .2 * in2mm, BOARD_DEPTH]);
        translate([.8 * in2mm, 4.6 * in2mm, 0])
            cube([.2 * in2mm, 1.8 * in2mm, BOARD_DEPTH]);
        translate([0, 6.2 * in2mm, 0])
            cube([1 * in2mm, .2 * in2mm, BOARD_DEPTH]);
    }
    translate([.1 * in2mm, 0, h]){
        translate([0, 2.3 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, 3.1 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, 3.5 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, 4.3 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
    }
    translate([.2 * in2mm, .1 * in2mm, h])
        cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
    translate([.9 * in2mm, 0, h]){
        translate([0, .1 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, 1.9 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, 2.3 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, 3.1 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, 3.5 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
    }
    translate([.1 * in2mm-BOARD_DEPTH/2 + 1, 4.7 * in2mm, -4]){
        translate([0, 0, 0]) { // top-right 
            cylinder(h = 8+BOARD_DEPTH, r = .95 * 1.27);
        }       
        translate([.8*in2mm/2, 0, 0]) { // top-middle 
            cylinder(h = 8+BOARD_DEPTH, r = .95 * 1.27);
        }
        translate([.8*in2mm, 0, 0]) { // top-left 
            cylinder(h = 8+BOARD_DEPTH, r = .95 * 1.27);
        }
        translate([0, 1.4*in2mm/2, 0]) { // middle-right 
            cylinder(h = 8+BOARD_DEPTH, r = .95 * 1.27);
        }
        translate([.8*in2mm, 1.2*in2mm/2, 0]) { // middle-left 
            cylinder(h = 8+BOARD_DEPTH, r = .95 * 1.27);
        }
        translate([0, 1.4*in2mm, 0]) { // bottom-right 
            cylinder(h = 8+BOARD_DEPTH, r = .95 * 1.27);
        }
        translate([.8*in2mm, 1.2*in2mm, 0]) { // bottom-left 
            cylinder(h = 8+BOARD_DEPTH, r = .95 * 1.27);
        }
        translate([(.8*in2mm)/2, 1.58*in2mm, 0]){
            translate([0, .05*in2mm,0]){
                cylinder(h = 8, d = .1*in2mm);
            }
            cube([.2*in2mm,.1 *in2mm,8]);
            translate([.2*in2mm, .05*in2mm,0]){
                cylinder(h = 8, d = .1*in2mm);
            }
        }
    }
}
translate([.1 * in2mm-BOARD_DEPTH/2, 4.7 * in2mm, 0]){
    translate([.8*in2mm/2,2,0]) { // Long crossbar
        cube([BOARD_DEPTH, 37, BOARD_DEPTH]);
    }
    translate([0,(41-BOARD_DEPTH)/2,0]) { // Short crossbar
        cube([.8*in2mm, BOARD_DEPTH, BOARD_DEPTH]);
    }
}