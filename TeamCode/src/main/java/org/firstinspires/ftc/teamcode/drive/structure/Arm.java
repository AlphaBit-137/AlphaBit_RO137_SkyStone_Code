
package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    public DcMotor arm;

    public ArmModes RobotArm = ArmModes.STOP;

    public enum ArmModes
    {
        STOP,
        UP,
        DOWN,
    }

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        arm = hwMap.get(DcMotorEx.class, "Arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setPower(0);
    }

    public void update(){
        switch (RobotArm){
            case UP:
                arm.setPower(0.5);
                break;
            case DOWN:
                arm.setPower(-0.5);
                break;
            case STOP:
                arm.setPower(0);
                break;
        }
    }

    public void switchToUP() {RobotArm = ArmModes.UP;}

    public void switchToOUT() {RobotArm = ArmModes.DOWN;}

    public void switchToSTOP() {RobotArm = ArmModes.STOP;}
}

