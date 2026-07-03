/*Sled for mounting a Nene flight computer in a BT-50 rocket tube

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
*/

$fa = .1;
$fs = 0.04;

BOARD_DEPTH = 2;

PUNCH_DEPTH = .1;
h = -1 * PUNCH_DEPTH / 2; //overhang

module velcro_hole() { // Values are converted from imperial
    translate([5.08,-1 * 5.08,h])
        cylinder(h=3.2, d=2.54);
    translate([5.08,0,h]) 
        cylinder(h=3.2, d=2.54);
    translate([3.81,-1*5.08,h]) 
        cube([2.54, 5.08, 3.2]);
}

module board(width, length) {
    difference() {
        translate([0, 0, -1 * BOARD_DEPTH / 2]){
            cube([width - PUNCH_DEPTH, length, BOARD_DEPTH]);
        }
    }
}

module round_corner(diameter) {
    difference() {
        translate([-2+h, -2+h, h/2]) {
            cube([4,4,BOARD_DEPTH+PUNCH_DEPTH]);
        }
        cylinder(h = BOARD_DEPTH, d = diameter);
        translate([0, -2, 0]) {
            cube([4,2,BOARD_DEPTH-h]);
        }
        translate([-2, 0, 0]) {
            cube([2,4,BOARD_DEPTH-h]);
        }
        translate([0, 0, 0]) {
            cube([2,2,BOARD_DEPTH-h]);
        }
    }
}

module brace_arm() {
    translate([4,-1,-1]){
        cube([2.85, 2.3, 10.5]);
        translate([0,1,8.5]){
            cube([2.85, 17, 2]);
            translate([0,17,0]){
                rotate([0,0,-45]){
                    cube([2.85, 8, 2]);
                }
            }
        }
    }
}

module mcu_brace() {
    // top square and peg
    translate([8.67,16.7,7.5]){
        cube([6,8,2]);
        translate([3,2.25-h,-1.6]){
            cylinder(h=3, d=2.1);
        }
    }

    // Right brace
    brace_arm();

    // Left brace
    translate([payload_width, 0, 0]){
        mirror([1, 0, 0]){
            brace_arm();
        }
    }
}

coupler_width = 21;
bulkhead_inset = 1.8;
bulkhead_depth = 1.27;
coupler_length = 50.73 - bulkhead_inset - bulkhead_depth;
payload_width = 23.4;
nosecone_shoulder = 18.85;
payload_length = 51.16 - nosecone_shoulder;

difference() {
    union() {
    board(payload_width, payload_length);
    translate([(payload_width - coupler_width)/2, payload_length, 0])
        board(coupler_width, coupler_length);
    }
    
    // Hole for MCU
    mcu_width = 18;
    translate([(payload_width - mcu_width)/2, (payload_width - mcu_width)/2, -1 * BOARD_DEPTH / 2-PUNCH_DEPTH]){
        cube([mcu_width, 40, BOARD_DEPTH+PUNCH_DEPTH*2]);
    }
    
    // Holes for sensor block screws
    translate([5.35, 74.395, -1 * BOARD_DEPTH / 2-PUNCH_DEPTH]){ // 5.85 determined by measuring in the render
        cylinder(h=BOARD_DEPTH + 2* PUNCH_DEPTH, d=2.5);
        translate([12.7, 0, 0]){ // .5"=12.7mm, .5" is from ICM20649 breakout spec... should be right?
            cylinder(h=BOARD_DEPTH + 2* PUNCH_DEPTH, d=2.5);
        }
    }
    
    // Cutouts for sensor block QWIIC connectors
    channel_width = 6.75; // Measured, the QWIIC connector itself is ~6.33mm
    translate([(payload_width-channel_width)/2, 72, -BOARD_DEPTH/2+h]){
        cube([channel_width, 8, BOARD_DEPTH/2]);
    }
    translate([(payload_width-channel_width)/2, 48, -BOARD_DEPTH/2+h]){
        cube([channel_width, 8, BOARD_DEPTH/2]);
    }
    
    // Hole for the velcro strap that holds the battery in place
    translate([(coupler_width-2.54)/2, 42, -1 * BOARD_DEPTH / 2-PUNCH_DEPTH]){
        rotate([0,0,90]){
            velcro_hole();
        }
    }
    
    // Round the corners so it doesn't get caught on glue
    translate([2,2,-BOARD_DEPTH/2]){
        round_corner(4); // Top right (MCU) corner
    }
    translate([payload_width-2-PUNCH_DEPTH,2,-BOARD_DEPTH/2]){
        rotate(a = 90, v=[0, 0, 180]){
            round_corner(4); // Top left (MCU) corner
        }
    }
    translate([coupler_width-.8-PUNCH_DEPTH,payload_length+coupler_length-2,-BOARD_DEPTH/2]){
        rotate(a = 180, v=[0, 0, 180]){
            round_corner(4); // Bottom left (battery) corner
        }
    }
    translate([3.2,payload_length+coupler_length-2,-BOARD_DEPTH/2]){
        rotate(a = 270, v=[0, 0, 180]){
            round_corner(4); // Bottom right (battery) corner
        }
    }
    translate([.7,payload_length-.7,-BOARD_DEPTH/2]){
        rotate(a = 270, v=[0, 0, 180]){
            round_corner(1.4); // Middle right (shoulder) corner
        }
    }
    translate([coupler_width+1.6, payload_length-.7,-BOARD_DEPTH/2]){
        rotate(a = 180, v=[0, 0, 180]){
            round_corner(1.4); // Middle left (shoulder) corner
        }
    }
}

ridge_width = 4.5; // Measured. Calculating gives .2"=5.08mm, but that's way too big?
ridge_length = 11.8;
ridge_height = 1.67; // Measured.

// Ridge to hold QWIIC Micro sensors straighst
translate([(payload_width-ridge_width)/2, 76.8-ridge_length, BOARD_DEPTH/2+h]){
    cube([ridge_width, ridge_length+PUNCH_DEPTH, ridge_height]);
    translate([10.9,ridge_length,0]){
        rotate([0,0,90]){
            cube([ridge_width*.7, ridge_length*1.5, ridge_height]);
        }
    }
}

// Patch gaps caused by corner rounding. This seems like a hack.
translate([1.2,30,-BOARD_DEPTH/2]){
    cube([(coupler_width-18)/2, 5, BOARD_DEPTH]); // right side
}
translate([20.7,30,-BOARD_DEPTH/2]){
    cube([(coupler_width-18)/2-.1, 5, BOARD_DEPTH]); // left side
}

mcu_brace();

/*
// Payload model
translate([payload_width/2,payload_width,0]){
    rotate([90, 0, 0]){
        difference() {
            cylinder(h=payload_width, d=24.1, center=false);
            translate([0,0,-1]){
                cylinder(h=payload_width+2, d=24.0, center=false);
            }
        }
    }
}
// MCU and radio model
translate([3.2,3.2,0]){
    cube([18,21.5,7.5]);
}
*/