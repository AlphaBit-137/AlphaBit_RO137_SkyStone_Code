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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID_Test", group = "Linear Opmode")

public class  PIDTest extends LinearOpMode {

    // Declaram obiectul robot cu clasa hardware si timpul de rulare
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx arm = null;
    public DcMotorEx lift = null;
    public Servo gripper = null;

    private static PIDFCoefficients Coefs = new PIDFCoefficients(1.174, 0.1174, 0, 11.74);

    //Constante

    @Override
    public void runOpMode() {

        arm = hardwareMap.get(DcMotorEx.class, "Extindere");
        gripper = hardwareMap.get(Servo.class, "Gripper");
        lift = hardwareMap.get(DcMotorEx.class, "Lift");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setVelocityPIDFCoefficients(6.0, 0.0, 0.0, 11.74);
        arm.setPositionPIDFCoefficients(5.0);

        gripper.setPosition(0.5);



        while(isStarted()==false && isStopRequested() == false) {
            arm.setTargetPosition(-250);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm.setPower(0.3);
            telemetry.addData("Encoder Count", arm.getCurrentPosition());
            telemetry.addData("Arm", arm.isBusy());
            telemetry.update();

        }

        //arm.setTargetPosition();
        waitForStart();

        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod
        while (opModeIsActive()) {

            double power = gamepad1.left_stick_y;


                    if(gamepad1.a){
                        arm.setTargetPosition(0);
                    }else{
                        if(gamepad1.y) {
                            arm.setTargetPosition(1000);
                        }
                        else{
                            arm.setTargetPosition(-250);
                        }

                    }

                    if(gamepad1.b){
                        gripper.setPosition(0);
                    }else{
                        gripper.setPosition(0.5);
                    }


            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            arm.setPower(0.3);
            lift.setPower(power);
            telemetry.addData("Encoder Count", arm.getCurrentPosition());
            telemetry.addData("Arm", arm.isBusy());
            telemetry.update();
        }
    }

}