// Reference: https://cdn-learn.adafruit.com/assets/assets/000/122/460/original/sensors_SG_10x5_fab_print.png

$fa = .1;
$fs = 0.04;

BOARD_DEPTH = 2;

punch_depth = .1;
h = -1 * punch_depth / 2;
bar_size = 6;

in2mm = 25.4;
p1in = .1*in2mm;
p15in = .15*in2mm;
p2in = .2*in2mm;
p3in = .3*in2mm;
p6in = .6*in2mm;

coupler_width = 95;
coupler_length = 199;
payload_width = 99.5;
payload_length = 70; // Total length: 177, max shoulder depth: 106.34mm


module board() {
    difference(){
        union(){
            cube([payload_width, payload_length, BOARD_DEPTH]);
            translate([(payload_width - coupler_width)/2, payload_length, 0])
                cube([coupler_width, 67, BOARD_DEPTH]);
            stabilizer_width = 7.5;
            stabilizer_length = 110;
            translate([payload_width / 2, -1 * stabilizer_length / 2 + .5, 1]) {
                cube([stabilizer_width, stabilizer_length, BOARD_DEPTH], center = true);
                translate([0, -1 * stabilizer_length / 2 + .5, 0]) {
                    cylinder(h = BOARD_DEPTH, d = stabilizer_width, center = true);
                }
            }
        }
        
    translate([bar_size, bar_size, h])
        cube([payload_width - 2*bar_size, payload_length - bar_size-h, BOARD_DEPTH+punch_depth]);
    
    translate([(payload_width - coupler_width)/2+bar_size, payload_length, h])
        cube([coupler_width - 2*bar_size, 67 - bar_size, BOARD_DEPTH+punch_depth]);
    }
}

module feather(){
    difference() {
        union(){
            translate([37.5, 0, 0]){
                cube([bar_size, 73, BOARD_DEPTH]);
                translate([0, 9, 0])
                    cube([19.5+bar_size, bar_size, BOARD_DEPTH]);
                translate([0, 50, 0])
                    cube([19.5+bar_size, bar_size, BOARD_DEPTH]);
            }
            translate([57, 0, 0]){
                cube([bar_size, 73, BOARD_DEPTH]);
                
            }
        }
        translate([(payload_width / 2) - (1.05*in2mm)/2, 5, h]){
            // FeatherS3D mounting holes
            translate([.2 * in2mm, 1.9 * in2mm, 0]){
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            }
            translate([.9 * in2mm, .1 * in2mm, 0]){
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            }
            translate([.9 * in2mm, 1.9 * in2mm, 0]){
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            }

            // Nene Battery cage mounting holes
            translate([3.4, 7, 0]){
                translate([0, 0, 0]) { // top-right 
                    cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
                }       
                translate([.8*in2mm/2, 0, 0]) { // top-middle 
                    cylinder(h = BOARD_DEPTH + punch_depth, r = .95 * 1.27);
                }
                translate([.8*in2mm, 0, 0]) { // top-left 
                    cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
                }
                translate([0, 1.4*in2mm/2, 0]) { // middle-right 
                    cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
                }
                translate([.8*in2mm, 1.2*in2mm/2, 0]) { // middle-left 
                    cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
                }
                translate([0, 1.4*in2mm, 0]) { // bottom-right 
                    cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
                }
                translate([.8*in2mm, 1.2*in2mm, 0]) { // bottom-left 
                    cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
                }
                translate([(.8*in2mm)/2, 1.58*in2mm, 0]){
                    translate([0, .05*in2mm,0]){
                        cylinder(h = BOARD_DEPTH+punch_depth, d = .1*in2mm);
                    }
                    cube([.2*in2mm,.1 *in2mm,BOARD_DEPTH+punch_depth]);
                    translate([.2*in2mm, .05*in2mm,0]){
                        cylinder(h = BOARD_DEPTH+punch_depth, d = .1*in2mm);
                    }
                }
            }
        }
    }
}

// 9DOF and GPS mounting holes
module gps(){
    difference() {
        union(){
            translate([0, 27, 0])
                cube([38, bar_size, BOARD_DEPTH]);
            translate([0, 47, 0])
                cube([38, bar_size, BOARD_DEPTH]);
        }
        translate([10, 30, h]){
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            translate([0, .8 * in2mm, 0])
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            translate([.8 * in2mm, 0, 0]){
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
                translate([0, .8 * in2mm, 0])
                    cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            }
        }
    }
}
    
module alti(){
    difference() {
        translate([62, 27, 0])
            cube([37, bar_size, BOARD_DEPTH]);
        // 6DOF and BMP581 mounting holes
        translate([70, 30, h]){
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            translate([.8 * in2mm, 0, 0]){
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            }
        }
    }
}

module blueraven(){
    difference() {
        union(){
            translate([33, 73, 0]){
                    cube([44, bar_size, BOARD_DEPTH]);
                translate([17, 0, 0])
                    cube([bar_size, 30, BOARD_DEPTH]);
                translate([38, 3, 0]){
                    cube([bar_size, 55, BOARD_DEPTH]);
                    translate([0, 7.5, 0])
                        cube([26, bar_size, BOARD_DEPTH]);
                    translate([0, 30, 0])
                        cube([26, bar_size, BOARD_DEPTH]);
                    translate([-8, 50, 0])
                        cube([32, bar_size, BOARD_DEPTH]);
                }
                translate([19, 24, 0])
                    cube([20, bar_size, BOARD_DEPTH]);
            }
        }
        // Blue Raven 2 mounting holes
        translate([40, 76, h]){
            cylinder(h=BOARD_DEPTH + 2 * punch_depth, d=2.5); // Top right
            translate([27, 0, 0])
                cylinder(h=BOARD_DEPTH + 2 * punch_depth, d=2.5); // Top left
            translate([27, 53.1, 0])
                cylinder(h=BOARD_DEPTH + 2 * punch_depth, d=2.5); // Bottom left
            translate([13.1, 23.9, 0])
                cylinder(h=BOARD_DEPTH + 2 * punch_depth, d=2.5); // Middle
        }

        // Blue Raven 2 (150mah) battery mounting holes
        translate([74,87.7,h]){
            translate([.2, 0, 0]) { // top-right 
                cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
            }
            translate([15.7, 0, 0]) { // top-left 
                cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
            }
            translate([.2, 41.5/2+.2, 0]) { // middle-right 
                cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
            }
            translate([15.7, 41.5/2+.2, 0]) { // middle-left 
                cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
            }
            translate([0.2, 41.5, 0]) { // bottom-right 
                cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
            }
            translate([15.7, 41.5, 0]) { // bottom-left 
                cylinder(h = BOARD_DEPTH+punch_depth, r = .95 * 1.27);
            }
            translate([15.7/2-2.5, -3, 0]){
                translate([0, .05*in2mm,0]){
                    cylinder(h = BOARD_DEPTH+punch_depth, d = .1*in2mm);
                }
                cube([.2*in2mm,.1 *in2mm,BOARD_DEPTH+punch_depth]);
                translate([.2*in2mm, .05*in2mm,0]){
                    cylinder(h = BOARD_DEPTH+punch_depth, d = .1*in2mm);
                }
            }
        }
    }
}

module buzzer(){
    translate([5, 67, 0])
        cube([18, 4.5, BOARD_DEPTH]);

    // Buzzer holder
    translate([20, 81.5, 0]){
        difference(){ 
           translate([0, 0, 0])
                cylinder(h = BOARD_DEPTH, d = 31.1);
           translate([0, 0, h])
                cylinder(h = BOARD_DEPTH+punch_depth*2, d = 20);
        }
    }
}
    
board();
feather();
gps();
alti();
blueraven();
buzzer();
translate([payload_width-8.2, payload_length-5.9, 0])
    cube([bar_size, bar_size, BOARD_DEPTH]); // Corner strengthening hack