$fa = .1;
$fs = 0.04;

DEPTH = 3;
EPSILON = .02;
I2M = 25.4;

batt_length = 41.5;
batt_width = 15.7;
peg_rad = .9 * 1.27;



module peg() {
    cylinder(h = 7+DEPTH, r = peg_rad);
}

// Walls
module battery_cover(){
    translate([-1,-5,9]) { // Top
        difference() {
            union(){
                difference() {
                    cube([batt_width+2*peg_rad, batt_length+4+2*peg_rad, DEPTH]);
                    translate([1.8, 2*peg_rad+4, -EPSILON/2]){
                        cube([.8*(batt_width+2*peg_rad), .8*(batt_length+2*peg_rad), DEPTH+EPSILON]);
                    }
                    translate([batt_width/2-1.3, 2, -EPSILON/2]){
                        translate([0, .05*I2M,0]){
                            cylinder(h = DEPTH + EPSILON, d = .1*I2M);
                        }
                        cube([.2*I2M,.1 *I2M,DEPTH+EPSILON]);
                        translate([.2*I2M, .05*I2M,0]){
                            cylinder(h = DEPTH + EPSILON, d = .1*I2M);
                        }
                    }
                    // 6.25 inner gap
                    // 8.08 outer gap
                    // 1.08 width = 1.15
                    // 1.57 height = 1.66         
                }
                translate([batt_width/2,5,0]) { // Long crossbar
                    cube([2.5, batt_length, DEPTH]);
                }
                translate([1,batt_length/2+4,0]) { // Short crossbar
                    cube([batt_width, 2.5, DEPTH]);
                }
            }
            // Troughs for battery fins to slide into
            translate([batt_width/2-3, -EPSILON/2, -EPSILON/2]){
                cube([1.15, batt_length+4, 1.66]);
                translate([7.12,0,0])
                    cube([1.15, batt_length+4, 1.66]);
            }
        }
    }

    // Pegs
    translate([.2, 0, -1]) { // top-right 
        peg();
    }

    translate([batt_width, 0, -1]) { // top-left 
        peg();
    }

    translate([.2, batt_length/2+.2, -1]) { // middle-right 
        peg();
    }

    translate([batt_width, batt_length/2+.2, -1]) { // middle-left 
        peg();
    }

    translate([0.2, batt_length, -1]) { // bottom-right 
        peg();
    }

    translate([batt_width, batt_length, -1]) { // bottom-left 
        peg();
    }
    
    
    // Battery block: Leave a 5mm gap, centered
    translate([batt_width/2, batt_length, 5.5]) {
        difference(){
            cube([batt_width, DEPTH, 7], center = true);
            cube([5+EPSILON, DEPTH+EPSILON, 7+EPSILON], center = true);
        }
    }
}

battery_cover();