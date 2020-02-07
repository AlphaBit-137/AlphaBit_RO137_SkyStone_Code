package org.firstinspires.ftc.teamcode.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {


    public static double NULL_POWER = 0d;
    public static double MAX_POWER = 0d;
    public static double MIN_POWER = 0d;
    public DcMotorEx leftWing = null;
    public DcMotorEx rightWing = null;

    public IntakeModes RobotIntake = IntakeModes.STOP;

    public enum IntakeModes {
        STOP,
        IN,
        OUT,
    }

    HardwareMap hwMap = null;
    public Gamepad gamepadA;
    public Gamepad gamepadB;
    public Telemetry telemetry;

    public Intake(Gamepad gamepadA, Gamepad gamepadB, Telemetry telemetry) {
        this.gamepadA = gamepadA;
        this.gamepadB = gamepadB;
        this.telemetry = telemetry;
    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        leftWing = hwMap.get(DcMotorEx.class, "Left_Wing");
        rightWing = hwMap.get(DcMotorEx.class, "Right_Wing");

        leftWing.setDirection(DcMotorEx.Direction.FORWARD);
        rightWing.setDirection(DcMotorEx.Direction.FORWARD);

        leftWing.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightWing.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftWing.setPower(0);
        rightWing.setPower(0);

    }

    void caseUpdate(){
        switch(RobotIntake){
            case STOP: {
                motorSetter(NULL_POWER);
                break;
            }

            case IN: {
                motorSetter(MAX_POWER);
                break;
            }

            case OUT:{
                motorSetter(MIN_POWER);
                break;
            }
        }
    }

    public void motorSetter (double power){
        leftWing.setPower(power);
        rightWing.setPower(power);
    }

    void update(){

        if(gamepadB.a){
            switchToIn();
        }
        if(gamepadB.y && RobotIntake == IntakeModes.IN){
            switchToStop();
        }
        if(gamepadB.y && RobotIntake == IntakeModes.STOP){
            switchToOut();
        }

        if(RobotIntake == Intake.IntakeModes.IN){
            telemetry.addData("Intake", "IN");
        }
        if(RobotIntake == Intake.IntakeModes.OUT){
            telemetry.addData("Intake", "OUT");
        }
        if(RobotIntake == Intake.IntakeModes.STOP){
            telemetry.addData("Intake", "STOP");
        }

        caseUpdate();
    }

    public void switchToStop(){ RobotIntake = IntakeModes.STOP;}
    public void switchToIn(){ RobotIntake = IntakeModes.IN;}
    public void switchToOut(){ RobotIntake = IntakeModes.OUT;}





}
