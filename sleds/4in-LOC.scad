// Reference: https://cdn-learn.adafruit.com/assets/assets/000/122/460/original/sensors_SG_10x5_fab_print.png

$fa = .1;
$fs = 0.04;

BOARD_LENGTH = 150;
BOARD_WIDTH = 98;
BOARD_DEPTH = 2;

punch_depth = .1;
h = -1 * punch_depth / 2;

in2mm = 25.4;
p1in = .1*in2mm;
p15in = .15*in2mm;
p2in = .2*in2mm;
p3in = .3*in2mm;
p6in = .6*in2mm;


module dot() {
    translate([0,0,h]) 
        cylinder(h=3.2, d=p1in);
}

module dash() {
    translate([p2in,-1 * p2in,h])
        cylinder(h=3.2, d=p1in);
    translate([p2in,0,h]) 
        cylinder(h=3.2, d=p1in);
    translate([p15in,-1*p2in,h]) 
        cube([p1in, p2in, 3.2]);
}

module swirly() {
    dot();
    for(i = [0 : 90 : 270])
        rotate([0,0,i])
            dash();
}

module wings() {
    difference() {
        translate([BOARD_WIDTH / 2, BOARD_LENGTH, 0]){
            rotate([90,0,0]){
                difference() {
                    cylinder(h = BOARD_LENGTH, d = BOARD_WIDTH);
                    translate([0, 0, h]){
                        cylinder(h = BOARD_LENGTH + punch_depth, d = BOARD_WIDTH-BOARD_DEPTH);
                    }
                }
            }
        }

        translate([0, h, 10]){
            cube([BOARD_WIDTH, BOARD_LENGTH+punch_depth, 999]);
        }
        
        translate([0, h, -10 - 999]){
            cube([BOARD_WIDTH, BOARD_LENGTH+punch_depth, 999]);
        }
    }
}

wings();
    
difference() {
    translate([-1 * h, 0, -1 * BOARD_DEPTH / 2]){
        cube([BOARD_WIDTH - punch_depth, BOARD_LENGTH, BOARD_DEPTH]);
    }
    translate([3, 6, -1 * BOARD_DEPTH / 2]){
        for(i = [p3in : p6in : BOARD_WIDTH - p3in])
            for(j = [p3in : p6in : BOARD_LENGTH - p3in])
                translate([i, j, 0]){
                    swirly();
                }
    }
}