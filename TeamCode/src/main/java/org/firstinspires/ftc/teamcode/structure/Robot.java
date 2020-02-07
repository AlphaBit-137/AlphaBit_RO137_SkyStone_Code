package org.firstinspires.ftc.teamcode.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.structure.Chassis;

public class Robot {

    private Chassis chassis;
    private Intake intake;
//    private arm arm;
//    private WhateverTheFuck wtf;

    public Robot(Gamepad gamepadA, Gamepad gamepadB, Telemetry telemetry) {
        this.chassis = new Chassis(gamepadA, gamepadB, telemetry);
        this.intake = new Intake(gamepadA, gamepadB, telemetry);
//      this.arm = new arm(gamepad);
//      his.wtf = new WhateverTheFuck(gamepad);
    }

    public void init(HardwareMap hardwareMap) {
        chassis.init(hardwareMap);
    }

    public void update() {
        chassis.update();
        intake.update();

//        arm.update();
//        wtf.update();
    }

}
