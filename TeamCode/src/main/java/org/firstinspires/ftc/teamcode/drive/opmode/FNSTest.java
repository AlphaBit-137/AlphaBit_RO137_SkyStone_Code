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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.structure.Outtake;

@TeleOp(name="FNSTest", group="Linear Opmode")

public class FNSTest extends LinearOpMode {

    // Declaram obiectul robot cu clasa hardware si timpul de rulare

    private ElapsedTime runtime = new ElapsedTime();
    Outtake outtake = new Outtake();



    //Constante

    @Override
    public void runOpMode() {

        outtake.init(hardwareMap);

        while(isStarted()==false && isStopRequested() == false) {
            outtake.switchToINIT();
            outtake.update(0);
        }

        waitForStart();
        int stoneNumber = 0;


        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod
        while (opModeIsActive()) {
            double controller = gamepad1.right_stick_y;


            if(gamepad1.x && stoneNumber == 0){
                stoneNumber = 1;
            }

            if(gamepad1.y && stoneNumber == 1){
                stoneNumber = 2;
            }


            if(gamepad1.x && stoneNumber == 2){
                stoneNumber = 1;
            }
            if(gamepad1.b && stoneNumber == 2){
                stoneNumber = 3;
            }


            if(gamepad1.y && stoneNumber == 3){
                stoneNumber = 2;
            }
            if(gamepad1.x && stoneNumber == 3){
                stoneNumber = 4;
            }

            if(gamepad1.y && stoneNumber == 4){
                stoneNumber = 5;
            }
            if (gamepad1.b && stoneNumber == 4){
                stoneNumber = 3;

            }

            if(gamepad1.b && stoneNumber == 5){
                stoneNumber = 6;
            }
            if (gamepad1.x && stoneNumber == 5){
                stoneNumber = 4;

            }

            if(gamepad1.x && stoneNumber == 6){
                stoneNumber = 7;
            }
            if (gamepad1.y && stoneNumber == 6){
                stoneNumber = 5;

            }


            if(gamepad1.y && stoneNumber == 7){
                stoneNumber = 8;
            }
            if (gamepad1.b && stoneNumber == 7){
                stoneNumber = 6;

            }

            if(gamepad1.b && stoneNumber == 8){
                stoneNumber = 9;
            }
            if (gamepad1.x && stoneNumber == 8){
                stoneNumber = 7;

            }

            if (gamepad1.y && stoneNumber == 9){
                stoneNumber = 8;

            }



            if(gamepad1.dpad_up && stoneNumber == 1){
                outtake.switchToSTONE_1();
            }
            if(gamepad1.dpad_up && stoneNumber == 2){
                outtake.switchToSTONE_2();
            }
            if(gamepad1.dpad_up && stoneNumber == 3){
                outtake.switchToSTONE_3();
            }
            if(gamepad1.dpad_up && stoneNumber == 4){
                outtake.switchToSTONE_4();
            }
            if(gamepad1.dpad_up && stoneNumber == 5){
                outtake.switchToSTONE_5();
            }
            if(gamepad1.dpad_up && stoneNumber == 6){
                outtake.switchToSTONE_6();
            }
            if(gamepad1.dpad_up && stoneNumber == 7){
                outtake.switchToSTONE_7();
            }
            if(gamepad1.dpad_up && stoneNumber == 8){
                outtake.switchToSTONE_8();
            }
            if(gamepad1.dpad_up && stoneNumber == 9){
                outtake.switchToSTONE_9();
            }


            if(gamepad1.right_stick_y != 0f){
                outtake.switchToFREE();
            }
            if(gamepad1.dpad_right){
                outtake.switchToGETSTONE();
            }
            if(gamepad1.dpad_left){
                outtake.gripper.switchToOPENED();
            }
            if(gamepad1.dpad_down){
                outtake.switchToRESET();
            }


            outtake.update(controller);
            telemetry.addData("Lift", outtake.lift.getLiftEncoder());
            telemetry.addData("Arm", outtake.arm.getArmEncoder());
            telemetry.addData("Stone", stoneNumber);
            telemetry.update();

        }

    }

}