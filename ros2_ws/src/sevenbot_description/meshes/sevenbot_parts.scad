// 7Bot robotarm — parametrische link-meshes
// Kinematische constanten uit officiële Arm7Bot.h firmware
// https://github.com/7Bot/7Bot-Arduino-lib/blob/master/Arm7Bot/src/Arm7Bot.h
//
// Render één link tegelijk via -D 'PART="<name>"', bijv:
//   openscad -D 'PART="upper_arm"' -o upper_arm_link.stl sevenbot_parts.scad
// Geldige PART waarden: base, shoulder, upper_arm, forearm,
//                       forearm_roll, wrist_pitch, wrist_roll, ee

$fn = 64;

// Link lengtes (mm) uit Arm7Bot.h
a = 120.0;    // upper arm
b = 40.0;     // shoulder offset
c = 198.50;   // forearm
d = 30.05;    // base radial offset (niet hier nodig — origin)
e = 77.80;    // (gripper plate, ongebruikt voor structurele links)
f = 22.10;    // wrist length
g = 12.0;     // end effector offset
h = 29.42;    // elbow offset

// Visueel
tube_d = 12;            // aluminium buisjes diameter
servo_box = [30, 30, 30]; // servo housing
plate_thick = 3;        // bracket plate dikte

// Materialen / kleuren (alleen preview)
ALU = "Silver";
BLK = "DimGray";
ORG = "DarkOrange";

// ==================== HELPERS ====================

// Aluminium buis: extends from origin along +Z, length L, diameter D
module alu_tube(L, D=tube_d) {
    color(ALU) cylinder(h=L, d=D);
}

// Servo housing block, centered on XY, base op origin (Z=0)
module servo_housing(size=servo_box) {
    color(BLK) translate([-size.x/2, -size.y/2, 0]) cube(size);
}

// Tube clamp: ring around an aluminium tube
module tube_clamp(D=tube_d, height=8, ring_thickness=2) {
    color(BLK) difference() {
        cylinder(h=height, d=D + 2*ring_thickness);
        translate([0, 0, -0.1]) cylinder(h=height + 0.2, d=D);
    }
}

// Yoke bracket — twee parallelle platen verbonden onderaan,
// houden iets vast tussen de platen.
module yoke_bracket(plate_w=30, plate_h=40, gap=20, thickness=plate_thick) {
    color(BLK) {
        // bottom plate
        translate([-plate_w/2, -gap/2 - thickness, 0])
            cube([plate_w, gap + 2*thickness, thickness]);
        // left plate
        translate([-plate_w/2, -gap/2 - thickness, 0])
            cube([plate_w, thickness, plate_h]);
        // right plate
        translate([-plate_w/2, gap/2, 0])
            cube([plate_w, thickness, plate_h]);
    }
}

// ==================== LINKS ====================

// BASE: cilindrische voet, geen joint, statisch
module base_link() {
    color(BLK) {
        // bodemplaat
        cylinder(h=8, d=120);
        // toren naar joint_0
        translate([0, 0, 8]) cylinder(h=52, d=60);
    }
}

// SHOULDER: kleine servo housing bovenop joint_0, joint_1 zit erop op hoogte b
module shoulder_link() {
    // servo box (joint_0 axis runs vertical through center)
    servo_housing([servo_box.x, servo_box.y, b]);
    // mount voor joint_1 axle aan top
    translate([0, 0, b]) rotate([90, 0, 0])
        color(BLK) cylinder(h=servo_box.y, d=12, center=true);
}

// UPPER_ARM: yoke bracket onderaan + alu tube van lengte a
module upper_arm_link() {
    // yoke houdt joint_2 axis op hoogte a
    yoke_bracket(plate_w=24, plate_h=20, gap=18);
    // hoofdtube
    translate([0, 0, 5]) alu_tube(L=a, D=tube_d);
    // bovenaan een tube clamp met joint_2 axis
    translate([0, 0, a]) tube_clamp();
}

// FOREARM: alu tube van lengte c met elbow servo housing aan begin
module forearm_link() {
    // elbow servo at base
    servo_housing();
    // forearm tube
    translate([0, 0, servo_box.z]) alu_tube(L=c - servo_box.z, D=tube_d);
    // tube clamp aan top
    translate([0, 0, c - 5]) tube_clamp();
}

// FOREARM_ROLL: korte cylinder van lengte h voor wrist roll-1
module forearm_roll_link() {
    color(BLK) cylinder(h=h, d=tube_d * 1.4);
}

// WRIST_PITCH: kleine kubische servo houder van lengte f
module wrist_pitch_link() {
    color(ORG) cube([24, 24, f], center=false);
    // center XY
    translate([-12, -12, 0]) {
        color(ORG) cube([24, 24, f]);
    }
}

// WRIST_ROLL: compacte eindrol van lengte g
module wrist_roll_link() {
    color(BLK) cylinder(h=g, d=20);
}

// EE: simpele gripper placeholder
module ee_link() {
    color(BLK) cube([30, 30, 40], center=false);
    translate([-15, -15, 0]) color(BLK) cube([30, 30, 40]);
}

// ==================== DISPATCH ====================

// Default = preview alle links naast elkaar
PART = "preview";

if (PART == "base")          base_link();
else if (PART == "shoulder") shoulder_link();
else if (PART == "upper_arm") upper_arm_link();
else if (PART == "forearm")  forearm_link();
else if (PART == "forearm_roll") forearm_roll_link();
else if (PART == "wrist_pitch")  wrist_pitch_link();
else if (PART == "wrist_roll")   wrist_roll_link();
else if (PART == "ee")           ee_link();
else {
    // preview: alles naast elkaar
    translate([0,    0, 0]) base_link();
    translate([180,  0, 0]) shoulder_link();
    translate([320,  0, 0]) upper_arm_link();
    translate([500,  0, 0]) forearm_link();
    translate([680,  0, 0]) forearm_roll_link();
    translate([780,  0, 0]) wrist_pitch_link();
    translate([880,  0, 0]) wrist_roll_link();
    translate([960,  0, 0]) ee_link();
}
