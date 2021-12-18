package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sliders {
    public DcMotor Slider = null;

    public Sliders.SlidersModes RobotSlider = Sliders.SlidersModes.STOP;


    public enum SlidersModes {
        IN,
        OUT,
        STOP,
    }

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        Slider = hwMap.get(DcMotorEx.class, "Slider");
        Slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slider.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Slider.setPower(0);
    }

    public void update(){
        switch (RobotSlider){
            case IN:{
                Slider.setPower(-0.6);
                break;
            }
            case OUT:{
                Slider.setPower(0.6);
                break;
            }
            case STOP:{
                Slider.setPower(0);
                break;
            }
        }
    }

    public void switchToIN() {RobotSlider = Sliders.SlidersModes.IN;}

    public void switchToOUT() {RobotSlider = Sliders.SlidersModes.OUT;}

    public void switchToSTOP() {RobotSlider = Sliders.SlidersModes.STOP;}

}

