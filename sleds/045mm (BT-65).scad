/*Sled for mounting a Nene flight computer in a 45mm rocket tube

--------------------------------------------------------------------------------
Copyright (C) 2026 Sam Procter

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
--------------------------------------------------------------------------------
*/

// High-res rendering
$fa = .1;
$fs = 0.04;

// Thickness of the board
BOARD_DEPTH = 2;

// How far voids should extend beyond the surface
punch_depth = .1;
h = -1 * punch_depth / 2;

bar_size = 5;

// Conversions of the imperial units used in the original design
in2mm = 25.4;

payload_width = 44.3;
payload_length = 83;
coupler_length = 60;
coupler_width = 37.5;

difference(){
    union() {
        difference(){
            cube([payload_width, payload_length, BOARD_DEPTH]);
            translate([bar_size, bar_size, h])
                cube([payload_width-2*bar_size, payload_length, BOARD_DEPTH+punch_depth]);
        }
        translate([9.43, 50, 0]){
            cube([bar_size, (payload_length + coupler_length)-50, BOARD_DEPTH]);
            translate([.8 * in2mm, 0, 0])
                cube([bar_size, (payload_length + coupler_length)-50, BOARD_DEPTH]);
            translate([0, 42, 0])
                cube([.99*in2mm, bar_size*2, BOARD_DEPTH]);
            translate([0, (payload_length + coupler_length)-50.5-bar_size, 0])
                cube([in2mm, bar_size+.5, BOARD_DEPTH]);// Thicken to avoid brittle wall
            translate([-bar_size, 0, 0]){
                cube([bar_size, bar_size, BOARD_DEPTH]);
                translate([0,28,0])
                    cube([bar_size, bar_size, BOARD_DEPTH]);
            }
            translate([.99*in2mm, 0, 0]){
                cube([bar_size, bar_size, BOARD_DEPTH]);
                translate([0,28,0])
                    cube([bar_size, bar_size, BOARD_DEPTH]);
            }
        }
        translate([11, 100, 0]){ // Crossbars between MCU and batteyr
            translate([.8*in2mm/2,2,0]) { // Long crossbar
                cube([BOARD_DEPTH, 37, BOARD_DEPTH]);
            }
            translate([0,19.5,0]) { // Short crossbar
                cube([.8*in2mm, BOARD_DEPTH, BOARD_DEPTH]);
            }
        }
        difference(){ // Buzzer holder
            translate([22, 16, 0])
                cylinder(h = BOARD_DEPTH, d = 30.6);
           translate([22, 16, h])
                cylinder(h = BOARD_DEPTH+punch_depth, d = 20);
        }
        translate([17, 31, 0])
            cube([26, bar_size, BOARD_DEPTH]);
        /*translate([6.5, 28.5, BOARD_DEPTH])
            #cube([15.2,10,2.9]); // Size of Boost Converter
        translate([9, 39.5, BOARD_DEPTH])
            #cube([in2mm, .7*in2mm, 1]); // Size of ICM20649*/
    }
    // FeatherS3D mounting holes
    translate([8.5, payload_length+9, h]){
        translate([.2 * in2mm, .1 * in2mm, 0]){
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        }
        translate([.9 * in2mm, 0, 0]){
            translate([0, .1 * in2mm, 0])
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            translate([0, 1.9 * in2mm, 0])
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        }
    }

    // Battery cage mounting holes
    translate([11.9, payload_length + 16, h]){
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
   
    
    // 9DOF and GPS mounting holes
    translate([11.9, 65, h]){
        cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([0, .8 * in2mm, 0])
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([.8 * in2mm, 0, 0]){
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
            translate([0, .8 * in2mm, 0])
                cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        }
    }

    // 6DOF and BMP581 mounting holes
    translate([11.9, 55, h]){
        cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        translate([.8 * in2mm, 0, 0]){
            cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
        }
    }
    
    // 12V boost converter mounting hole
    translate([20, 33.5, h])
        cylinder(h=BOARD_DEPTH + 2* punch_depth, d=2.5);
}