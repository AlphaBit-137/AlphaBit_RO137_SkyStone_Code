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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.drive.structure.Chassis;
import org.firstinspires.ftc.teamcode.drive.structure.Claws;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Outtake;


@TeleOp(name="Op_Mode", group="Linear Opmode")

public class OPMode extends LinearOpMode {

    // Declaram obiectul robot cu clasa hardware si timpul de rulare

    private ElapsedTime runtime = new ElapsedTime();
    Chassis chassis = new Chassis();
    Claws claws = new Claws();
    Outtake outtake = new Outtake();
    Intake intake = new Intake();

    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap);
        claws.init(hardwareMap);
        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        while(isStarted()==false && isStopRequested() == false) {
            outtake.switchToINIT();
            intake.switchToSTOP();
            claws.switchToOPENED();



            outtake.update(0);
            intake.update();
            claws.update();
        }


        waitForStart();
        int stoneNumber = 0;

        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod

        while (opModeIsActive()) {


            //Driver 1 ++++++++++++++++++++++++++++++++++++++++++++++++++++++

            double Front, Turn, Drive1, Drive2, Drive3, Drive4, strafeLeft, strafeRight;

            //Primirea datelor de la joystick-uri
            Front = gamepad1.left_stick_y;
            Turn = gamepad1.right_stick_x;
            strafeLeft = gamepad1.left_trigger;
            strafeRight = gamepad1.right_trigger;


            Drive1 = Range.clip(Front - Turn, -1.0, 1.0);
            Drive2 = Range.clip(Front - Turn, -1.0, 1.0);
            Drive3 = Range.clip(Front + Turn, -1.0, 1.0);
            Drive4 = Range.clip(Front + Turn, -1.0, 1.0);


            //Calcularea puterii redate motoarelor
            if(gamepad1.left_stick_y != 0f || gamepad1.right_stick_x  != 0f)
            {
                chassis.update(Drive1, Drive2, Drive3, Drive4);
            }else if(gamepad1.left_trigger != 0f || gamepad1.right_trigger != 0f){

                if(gamepad1.left_trigger != 0f){
                    chassis.update(strafeLeft, -strafeLeft, strafeLeft, -strafeLeft);
                }

                if(gamepad1.right_trigger != 0f){
                    chassis.update(-strafeRight, strafeRight, -strafeRight, strafeRight);
                }
            }else{
                chassis.update(NULL_POWER, NULL_POWER, NULL_POWER, NULL_POWER);
            }


            //Chassis Speed
            if(gamepad1.a){
                chassis.switchToFast();
            }
            if(gamepad1.b){
                chassis.switchToSlow();
            }

            //Claws
            if(gamepad1.left_bumper){
                claws.switchToOPENED();
            }
            if(gamepad1.right_bumper){
                claws.switchToCLOSED();
            }


            //Driver 2 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            double secondController = gamepad2.right_stick_y;


            //Stone Cycling
            if(gamepad2.x && stoneNumber == 0){
                stoneNumber = 1;
            }

            if(gamepad2.y && stoneNumber == 1){
                stoneNumber = 2;
            }


            if(gamepad2.x && stoneNumber == 2){
                stoneNumber = 1;
            }
            if(gamepad2.b && stoneNumber == 2){
                stoneNumber = 3;
            }


            if(gamepad2.y && stoneNumber == 3){
                stoneNumber = 2;
            }
            if(gamepad2.x && stoneNumber == 3){
                stoneNumber = 4;
            }

            if(gamepad2.y && stoneNumber == 4){
                stoneNumber = 5;
            }
            if (gamepad2.b && stoneNumber == 4){
                stoneNumber = 3;

            }

            if(gamepad2.b && stoneNumber == 5){
                stoneNumber = 6;
            }
            if (gamepad2.x && stoneNumber == 5){
                stoneNumber = 4;

            }

            if(gamepad2.x && stoneNumber == 6){
                stoneNumber = 7;
            }
            if (gamepad2.y && stoneNumber == 6){
                stoneNumber = 5;

            }


            if(gamepad2.y && stoneNumber == 7){
                stoneNumber = 8;
            }
            if (gamepad2.b && stoneNumber == 7){
                stoneNumber = 6;

            }

            if(gamepad2.b && stoneNumber == 8){
                stoneNumber = 9;
            }
            if (gamepad2.x && stoneNumber == 8){
                stoneNumber = 7;

            }

            if (gamepad2.y && stoneNumber == 9){
                stoneNumber = 8;

            }


            //Outtake Movements

            if(gamepad2.dpad_up && stoneNumber == 1){
                outtake.switchToSTONE_1();
            }
            if(gamepad2.dpad_up && stoneNumber == 2){
                outtake.switchToSTONE_2();
            }
            if(gamepad2.dpad_up && stoneNumber == 3){
                outtake.switchToSTONE_3();
            }
            if(gamepad2.dpad_up && stoneNumber == 4){
                outtake.switchToSTONE_4();
            }
            if(gamepad2.dpad_up && stoneNumber == 5){
                outtake.switchToSTONE_5();
            }
            if(gamepad2.dpad_up && stoneNumber == 6){
                outtake.switchToSTONE_6();
            }
            if(gamepad2.dpad_up && stoneNumber == 7){
                outtake.switchToSTONE_7();
            }
            if(gamepad2.dpad_up && stoneNumber == 8){
                outtake.switchToSTONE_8();
            }
            if(gamepad2.dpad_up && stoneNumber == 9){
                outtake.switchToSTONE_9();
            }



            if(gamepad2.right_stick_y != 0f){
                outtake.switchToFREE();
            }
            if(gamepad2.dpad_right){
                outtake.switchToGETSTONE();
            }
            if(gamepad2.dpad_left){
                outtake.gripper.switchToOPENED();
            }
            if(gamepad2.dpad_down){
                outtake.switchToRESET();
            }


            if(gamepad2.left_bumper){
                intake.switchToIN();
            } else if (gamepad2.right_bumper) {
                intake.switchToOUT();
            }else{
                intake.switchToSTOP();
            }



            //Chassis
            if(chassis.RobotChasis == Chassis.ChassisModes.SLOW){
                telemetry.addData("Chassis", "SLOW");
            }
            if(chassis.RobotChasis == Chassis.ChassisModes.FAST){
                telemetry.addData("Chassis", "FAST");
            }

            //Claws
            if(claws.RobotClaw == Claws.ClawModes.OPENED){
                telemetry.addData("Claws", "OPENED");
            }
            if(claws.RobotClaw == Claws.ClawModes.CLOSED){
                telemetry.addData("Claws", "CLOSED");
            }

            //Stone Level
            if(stoneNumber == 0){
                telemetry.addData("Stone Level", "NULL");
            }
            if(stoneNumber == 1){
                telemetry.addData("Stone Level", "STONE_1");
            }
            if(stoneNumber == 2){
                telemetry.addData("Stone Level", "STONE_2");
            }
            if(stoneNumber == 3){
                telemetry.addData("Stone Level", "STONE_3");
            }
            if(stoneNumber == 4){
                telemetry.addData("Stone Level", "STONE_4");
            }
            if(stoneNumber == 5){
                telemetry.addData("Stone Level", "STONE_5");
            }
            if(stoneNumber == 6){
                telemetry.addData("Stone Level", "STONE_6");
            }
            if(stoneNumber == 7){
                telemetry.addData("Stone Level", "STONE_7");
            }
            if(stoneNumber == 8){
                telemetry.addData("Stone Level", "STONE_8");
            }
            if(stoneNumber == 9){
                telemetry.addData("Stone Level", "STONE_9");
            }

            //Intake
            if(intake.RobotIntake == Intake.IntakeModes.IN){
                telemetry.addData("Intake", "IN");
            }
            if(intake.RobotIntake == Intake.IntakeModes.OUT){
                telemetry.addData("Intake", "OUT");
            }
            if(intake.RobotIntake == Intake.IntakeModes.STOP){
                telemetry.addData("Intake", "STOP");
            }


            telemetry.update();
            claws.update();
            outtake.update(secondController);
            intake.update();
        }
    }

}