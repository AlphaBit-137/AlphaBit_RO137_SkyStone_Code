package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotorEx intake = null;

    public IntakeModes RobotIntake = IntakeModes.STOP;

    public enum IntakeModes
    {
        IN,
        OUT,
        STOP
    }

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        intake = hwMap.get(DcMotorEx.class, "Intake");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
    }

    public void update(){
        switch (RobotIntake){
            case IN:{
                intake.setPower(-1.0);
                break;
            }
            case OUT:{
                intake.setPower(1.0);
                break;
            }
            case STOP:{
                intake.setPower(0);
                break;
            }
        }
    }

    public void switchToIN() {RobotIntake = IntakeModes.IN;}

    public void switchToOUT() {RobotIntake = IntakeModes.OUT;}

    public void switchToSTOP() {RobotIntake = IntakeModes.STOP;}
}
