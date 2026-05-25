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

coupler_width = 23;
bulkhead_inset = 1.8;
bulkhead_depth = 1.27;
coupler_length = 50.73 - bulkhead_inset - bulkhead_depth;
payload_width = 24.4;
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
        cube([mcu_width, 35, BOARD_DEPTH+PUNCH_DEPTH*2]);
    }
    
    // Holes for sensor block screws
    translate([5.85, 50, -1 * BOARD_DEPTH / 2-PUNCH_DEPTH]){ // 5.85 determined by measuring in the render
        cylinder(h=BOARD_DEPTH + 2* PUNCH_DEPTH, d=2.5);
        translate([12.7, 0, 0]){ // .5"=12.7mm, .5" is from ICM20649 breakout spec... should be right?
            cylinder(h=BOARD_DEPTH + 2* PUNCH_DEPTH, d=2.5);
        }
    }
    
    // Channel for sensor block QWIIC connector
    channel_width = 6.75; // Measured, the QWIIC connector itself is ~6.33mm
    translate([(payload_width-channel_width)/2, 15, -BOARD_DEPTH/2+h]){
        cube([channel_width, 50, BOARD_DEPTH/2]);
    }
    
    // Hole for battery
    batt_width = 7.25;
    batt_length = 20;
    translate([(payload_width - batt_width)/2, 58, -1 * BOARD_DEPTH / 2-PUNCH_DEPTH]){
        cube([batt_width, batt_length, BOARD_DEPTH+PUNCH_DEPTH*2]);
    }
    // Holes for the velcro straps that hold the battery in place
    translate([0, 70, -1 * BOARD_DEPTH / 2-PUNCH_DEPTH]){
        velcro_hole();
        translate([14.5, 0, 0]){
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
    translate([coupler_width-1.3-PUNCH_DEPTH,payload_length+coupler_length-2,-BOARD_DEPTH/2]){
        rotate(a = 180, v=[0, 0, 180]){
            round_corner(4); // Bottom left (battery) corner
        }
    }
    translate([2.7,payload_length+coupler_length-2,-BOARD_DEPTH/2]){
        rotate(a = 270, v=[0, 0, 180]){
            round_corner(4); // Bottom right (battery) corner
        }
    }
    translate([.7,payload_length-1,-BOARD_DEPTH/2]){
        rotate(a = 270, v=[0, 0, 180]){
            round_corner(1.4); // Middle right (shoulder) corner
        }
    }
    translate([coupler_width+.6, payload_length-1,-BOARD_DEPTH/2]){
        rotate(a = 180, v=[0, 0, 180]){
            round_corner(1.4); // Middle left (shoulder) corner
        }
    }
}

ridge_width = 3.6; // Measured. Calculating gives .2"=5.08mm, but that's way too big?
ridge_length = 19.7; // Measured. Specs give .75"=19.05mm
ridge_height = 1.67; // Measured.

// Ridge to hold QWIIC Micro sensors strait
translate([(payload_width-ridge_width)/2, 50-ridge_length, BOARD_DEPTH/2+h]){
    cube([ridge_width, ridge_length, ridge_height]);
}

// Patch gaps caused by corner rounding. This seems like a hack.
translate([.7,30,-BOARD_DEPTH/2]){
    cube([(coupler_width-18)/2, 5, BOARD_DEPTH]); // right side
}
translate([21.2,30,-BOARD_DEPTH/2]){
    cube([(coupler_width-18)/2-.1, 5, BOARD_DEPTH]); // right side
}
