package org.firstinspires.ftc.teamcode.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.Chassis;

public class Robot {

    private Chassis chassis;
//    private Arm arm;
//    private WhateverTheFuck wtf;

    public Robot(Gamepad gamepad, Telemetry telemetry) {
        this.chassis = new Chassis(gamepad, telemetry);
//        this.arm = new Arm(gamepad);
//        this.wtf = new WhateverTheFuck(gamepad);
    }

    public void init(HardwareMap hardwareMap) {
        chassis.init(hardwareMap);
    }

    public void update() {
        chassis.update();

//        arm.update();
//        wtf.update();
    }

}
