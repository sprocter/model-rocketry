// Reference: https://cdn-learn.adafruit.com/assets/assets/000/122/460/original/sensors_SG_10x5_fab_print.png

// High-res rendering
$fa = .1;
$fs = 0.04;

// Thickness of the board
BOARD_DEPTH = 2;

// How far voids should extend beyond the surface
punch_depth = .1;
h = -1 * punch_depth / 2;

// Conversions of the imperial units used in the original design
in2mm = 25.4;
p1in = .1*in2mm;
p15in = .15*in2mm;
p2in = .2*in2mm;
p3in = .3*in2mm;
p6in = .6*in2mm;

// The dot in the middle of the swirlies
module dot() {
    translate([0,0,h]) 
        cylinder(h=3.2, d=p1in);
}

// The rounded rectangles that border the swirlies
module dash() {
    translate([p2in,-1 * p2in,h])
        cylinder(h=3.2, d=p1in);
    translate([p2in,0,h]) 
        cylinder(h=3.2, d=p1in);
    translate([p15in,-1*p2in,h]) 
        cube([p1in, p2in, 3.2]);
}

// Used to render one swirly
module swirly() {
    dot();
    for(i = [0 : 90 : 270])
        rotate([0,0,i])
            dash();
}

// Used to render the round edges which hold the sled in the payload
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

// Simplified version of the round edges which hold the sled in the payload, with added percent parameter to control how much of the wing to inclde.
// percent should be float between 0 and 1, percent of the width to remove from the wings. The value 1 means the wings are the full cylinder
module wings2(width, length,percent) {

    
    //Walls
    difference() {
        cylinder(h = length, d = width);            
        cylinder(h = length + punch_depth, d = width-BOARD_DEPTH);
            cube([width, width*percent, 999],center=true);
     }

};


// The board is the flat part
module board(payload_width, payload_length, aft_overhang_length, aft_overhang_width) {
    //wings(payload_width, payload_length);        
    translate([-1 * h, 0, -1 * BOARD_DEPTH / 2]){
        cube([payload_width - punch_depth, payload_length, BOARD_DEPTH]);
    }
    translate([-1 * h + (payload_width-aft_overhang_width)/2, payload_length, -1 * BOARD_DEPTH / 2]){
        cube([aft_overhang_width - punch_depth, aft_overhang_length, BOARD_DEPTH]);
    }
}

payload_width = 44.3;
payload_length = 83;
aft_overhang_length = 60;
aft_overhang_width = 37.5;

// This module removes the swirlies from the board and adds some tweaks for the battery
module extended_board () {
    difference() {
            board(payload_width, payload_length, aft_overhang_length, aft_overhang_width);
            translate([-0.5, -10, -1 * BOARD_DEPTH / 2]){
                for(i = [p3in : p6in : payload_width])
                    for(j = [p3in : p6in : payload_length + aft_overhang_length + p3in])
                        translate([i, j, 0]){
                            swirly();
                        }
            }
            // Add a hole for the battery plug to go through
            translate([21.2, 98, -1*BOARD_DEPTH]){
                cube([7.5,7.5,BOARD_DEPTH*2]);
            }
    }
    
    // Block some swirly elements that are close to where we need to have a velcro strap
    translate([35.85, 97.8, -.5*BOARD_DEPTH]){
        cube([3,10,BOARD_DEPTH]);
    }
}

module structural_beam(){
    intersection(){
        cube([BOARD_DEPTH*2,payload_width,BOARD_DEPTH*2],center=true);
        cylinder(h=BOARD_DEPTH,d=payload_width);
    };
}


module wing_block() {
        //top mount - for structural support
        // Structural base
        
        difference() {
            union(){    
                wings2(payload_width, payload_length,.8);
                structural_beam();
            };
            translate([0,-(payload_width)/2+.025,0]) {
                rotate([90,0,90]) {
                    board(payload_width,payload_length,0);
                };
            };
       };

}


// Setup to output two different stl files - the wings and the boards
// Only the wings
//wing_block();

//Only the board
difference() {
    extended_board();
    // Add a "dash" so the velcro strap fits nicely
    translate([35, 104, -.5*BOARD_DEPTH]){
        dash();
    }
}