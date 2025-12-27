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

module wings(width, length) {
    difference() {
        translate([width / 2, length, 0]){
            rotate([90,0,0]){
                difference() {
                    cylinder(h = length, d = width);
                    translate([0, 0, h]){
                        cylinder(h = length + punch_depth, d = width-BOARD_DEPTH);
                    }
                }
            }
        }

        translate([0, h, 10]){
            cube([width, length+punch_depth, 999]);
        }
        
        translate([0, h, -10 - 999]){
            cube([width, length+punch_depth, 999]);
        }
    }
}

module board(width, length) {
    wings(width, length);        
    difference() {
        translate([-1 * h, 0, -1 * BOARD_DEPTH / 2]){
            cube([width - punch_depth, length, BOARD_DEPTH]);
        }
        translate([3, 6, -1 * BOARD_DEPTH / 2]){
            for(i = [p3in : p6in : width - p3in])
                for(j = [p3in : p6in : length - p3in])
                    translate([i, j, 0]){
                        swirly();
                    }
        }
    }
}


coupler_width = 95;
coupler_length = 194;
payload_width = 97.5;
payload_length = 127.5;

board(payload_width, payload_length);
translate([(payload_width - coupler_width)/2, payload_length, 0])
    board(coupler_width, coupler_length);