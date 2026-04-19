$fa = .1;
$fs = 0.04;

DEPTH = 2.413;
EPSILON = .02;
I2M = 25.4;


module peg() {
    cylinder(h = 8+DEPTH, r = .95 * 1.27);
}

// Walls
module battery_cover(){
    translate([-DEPTH/2,-DEPTH/2,8+DEPTH]) { // Top
        difference() {
            cube([.8*I2M+DEPTH, 43+DEPTH/2, DEPTH]);
            translate([DEPTH, DEPTH, -EPSILON/2]){
                cube([.8*I2M+DEPTH-2*DEPTH, 43-2*DEPTH-2, DEPTH+EPSILON]);
            }
            translate([.8*I2M/2, 1.6*I2M, -EPSILON/2]){
                translate([0, .05*I2M,0]){
                    cylinder(h = DEPTH + EPSILON, d = .1*I2M);
                }
                cube([.2*I2M,.1 *I2M,DEPTH+EPSILON]);
                translate([.2*I2M, .05*I2M,0]){
                    cylinder(h = DEPTH + EPSILON, d = .1*I2M);
                }
            }
        }
        translate([.8*I2M/2,0,0]) { // Long crossbar
            cube([DEPTH, 40, DEPTH]);
        }
        translate([0,(43-DEPTH)/2,0]) { // Short crossbar
            cube([.8*I2M, DEPTH, DEPTH]);
        }
    }

    // Pegs

    translate([0, 0, 0]) { // top-right 
        peg();
    }

    translate([.8*I2M/2, 0, 0]) { // top-middle 
        peg();
    }

    translate([.8*I2M, 0, 0]) { // top-left 
        peg();
    }

    translate([0, 1.4*I2M/2, 0]) { // middle-right 
        peg();
    }

    translate([.8*I2M, 1.2*I2M/2, 0]) { // middle-left 
        peg();
    }

    translate([0, 1.4*I2M, 0]) { // bottom-right 
        peg();
    }

    translate([.8*I2M, 1.2*I2M, 0]) { // bottom-left 
        peg();
    }
}

battery_cover();