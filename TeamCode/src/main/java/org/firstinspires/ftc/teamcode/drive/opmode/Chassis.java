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

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.opmode.OPMode;

import java.security.KeyStore;

public class Chassis
{
    /* Public OpMode members. */

    public DcMotor  RightFront   = null;    //Config: 0
    public DcMotor  RightBack  = null;      //Config: 1
    public DcMotor  LeftBack = null;        //Config: 2
    public DcMotor  LeftFront = null;       //Config: 3

    public OPMode test;


    public ChassisModes  RobotChasis = ChassisModes.FAST;

    public enum ChassisModes{
            FAST,
            SLOW,
    }

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */
    public Chassis(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        LeftBack = hwMap.get(DcMotor.class, "Left_Back");
        RightFront = hwMap.get(DcMotor.class, "Right_Front");
        LeftFront = hwMap.get(DcMotor.class, "Left_Front");
        RightBack = hwMap.get(DcMotor.class, "Right_Back");

        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        LeftBack.setPower(0);
        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void update(double D1, double D2, double D3, double D4) {
        double RightFrontPower;
        double RightBackPower;
        double LeftBackPower;
        double LeftFrontPower;



        switch (RobotChasis) {
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
                    RightFrontPower = Range.clip(D4, -0.2,0.2);
                    MotorSetter(LeftFrontPower, LeftBackPower, RightBackPower, RightFrontPower);
                    break;

                }
            }
    }

    public void switchToFast(){
        RobotChasis=ChassisModes.FAST;
    }
    public void switchToSlow(){
        RobotChasis=ChassisModes.SLOW;
    }




    public void MotorSetter(double x1, double x2, double x3, double x4){
        LeftFront.setPower(x1);
        LeftBack.setPower(x2);
        RightBack.setPower(x3);
        RightFront.setPower(x4);

    }



 }

