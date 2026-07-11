$fa = .1;
$fs = 0.04;

DEPTH = 2.413;
EPSILON = .02;
I2M = 25.4;


module peg() {
    cylinder(h = 7+DEPTH, r = .95 * 1.27);
}

// Walls
module battery_cover(){
    translate([-DEPTH/2,-DEPTH/2,7+DEPTH]) { // Top
        difference() {
            cube([.8*I2M+DEPTH, 43+DEPTH/2, DEPTH-.413]);
            translate([DEPTH, DEPTH, -EPSILON/2]){
                cube([.8*I2M+DEPTH-2*DEPTH, 43-2*DEPTH-2, DEPTH+EPSILON]);
            }
            translate([.9*I2M/2, 1.6*I2M, -EPSILON/2]){
                translate([0, .05*I2M,0]){
                    cylinder(h = DEPTH + EPSILON, d = .1*I2M);
                }
                cube([.2*I2M,.1 *I2M,DEPTH+EPSILON]);
                translate([.2*I2M, .05*I2M,0]){
                    cylinder(h = DEPTH + EPSILON, d = .1*I2M);
                }
            }
            // Beveled corners
            rotate([0,45,0]){
                translate([-4.5,-1 * EPSILON,0])
                    cube([4, 45, 4]);
                translate([12.5,-1 * EPSILON,16.5])
                    cube([4, 45, 4]);
            }
        }
        translate([.8*I2M/2,0,0]) { // Long crossbar
            cube([DEPTH, 40, DEPTH-.413]);
        }
        translate([2,(43-DEPTH)/2,0]) { // Short crossbar
            cube([.73*I2M, DEPTH, DEPTH-.413]);
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