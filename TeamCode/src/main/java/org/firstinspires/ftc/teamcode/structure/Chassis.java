/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Chassis {
    /* Public OpMode members. */

    public DcMotorEx rightFront = null;    //Config: 0 vechi FR
    public DcMotorEx rightBack = null;      //Config: 1 vechi BR
    public DcMotorEx leftBack = null;        //Config: 2 vechi BL
    public DcMotorEx leftFront = null;       //Config: 3 vechi FL



    public ChassisModes RobotChassis = ChassisModes.FAST;

    public enum ChassisModes {
        FAST,
        SLOW,
    }

    /* local OpMode members. */
    HardwareMap hwMap = null;

    public Gamepad gamepadA;
    public Gamepad gamepadB;
    public Telemetry telemetry;

    /* Constructor */
    public Chassis(Gamepad gamepad1, Gamepad gamepad2 , Telemetry telemetry) {
        this.gamepadA = gamepad1;
        this.gamepadB = gamepad2;
        this.telemetry = telemetry;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftBack = hwMap.get(DcMotorEx.class, "Back_Left");
        rightFront = hwMap.get(DcMotorEx.class, "Front_Right");
        leftFront = hwMap.get(DcMotorEx.class, "Front_Left");
        rightBack = hwMap.get(DcMotorEx.class, "Back_Right");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void update() {
        double NULL_POWER = 0d;

        //Primirea datelor de la joystick-uri
        double front = gamepadA.left_stick_y;
        double turn = gamepadA.right_stick_x;
        double strafeLeft = gamepadA.left_trigger;
        double strafeRight = gamepadA.right_trigger;

        //Calcularea puterii redate motoarelor
        if (gamepadA.left_stick_y != 0f || gamepadA.right_stick_x != 0f) {
            double Drive1 = Range.clip(front - turn, -1.0, 1.0);
            double Drive2 = Range.clip(front - turn, -1.0, 1.0);
            double Drive3 = Range.clip(front + turn, -1.0, 1.0);
            double Drive4 = Range.clip(front + turn, -1.0, 1.0);
            caseUpdate(Drive1, Drive2, Drive3, Drive4);

        } else if (gamepadA.left_trigger != 0f || gamepadA.right_trigger != 0f) {
            if (gamepadA.left_trigger != 0f) {
                caseUpdate(strafeLeft, -strafeLeft, strafeLeft, -strafeLeft);
            }
            if (gamepadA.right_trigger != 0f) {
                caseUpdate(-strafeRight, strafeRight, -strafeRight, strafeRight);
            }
        } else {
            caseUpdate(NULL_POWER, NULL_POWER, NULL_POWER, NULL_POWER);
        }


        if(gamepadA.a){
            this.switchToFast();
        }
        if(gamepadA.b){
            this.switchToSlow();
        }

        if(RobotChassis == Chassis.ChassisModes.SLOW){
            telemetry.addData("Chassis", "SLOW");
        }
        if(RobotChassis == Chassis.ChassisModes.FAST){
            telemetry.addData("Chassis", "FAST");
        }


    }

    private void caseUpdate(double D1, double D2, double D3, double D4) {
        double RightFrontPower;
        double RightBackPower;
        double LeftBackPower;
        double LeftFrontPower;


        switch (RobotChassis) {
            case FAST: {
                LeftFrontPower = Range.clip(D1, -1, 1);
                LeftBackPower = Range.clip(D2, -1, 1);
                RightBackPower = Range.clip(D3, -1, 1);
                RightFrontPower = Range.clip(D4, -1, 1);
                MotorSetter(LeftFrontPower, LeftBackPower, RightBackPower, RightFrontPower);
                break;
            }
            case SLOW: {
                LeftFrontPower = Range.clip(D1, -0.2, 0.2);
                LeftBackPower = Range.clip(D2, -0.2, 0.2);
                RightBackPower = Range.clip(D3, -0.2, 0.2);
                RightFrontPower = Range.clip(D4, -0.2, 0.2);
                MotorSetter(LeftFrontPower, LeftBackPower, RightBackPower, RightFrontPower);
                break;

            }
        }
    }

    public void switchToFast() {
        RobotChassis = ChassisModes.FAST;
    }
    public void switchToSlow() { RobotChassis = ChassisModes.SLOW; }


    public void MotorSetter(double x1, double x2, double x3, double x4) {
        leftFront.setPower(x1);
        leftBack.setPower(x2);
        rightBack.setPower(x3);
        rightFront.setPower(x4);

    }


}

