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


@TeleOp(name="Op_Mode_Test", group="Linear Opmode")

public class OPModeTest extends LinearOpMode {

    // Declaram obiectul robot cu clasa hardware si timpul de rulare

    private ElapsedTime runtime = new ElapsedTime();
    Chassis DriveTrain = new Chassis();


    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

    @Override
    public void runOpMode() {
        DriveTrain.init(hardwareMap);
        waitForStart();

        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod

        while (opModeIsActive()) {
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
                DriveTrain.update(Drive1, Drive2, Drive3, Drive4);
            }else if(gamepad1.left_trigger != 0f || gamepad1.right_trigger != 0f){

                    if(gamepad1.left_trigger != 0f){
                        DriveTrain.update(strafeLeft, -strafeLeft, strafeLeft, -strafeLeft);
                    }

                    if(gamepad1.right_trigger != 0f){
                        DriveTrain.update(-strafeRight, strafeRight, -strafeRight, strafeRight);
                    }
            }else{
                DriveTrain.update(NULL_POWER, NULL_POWER, NULL_POWER, NULL_POWER);
            }


            if(gamepad1.a){
                DriveTrain.switchToFast();
            }
            if(gamepad1.b){
                DriveTrain.switchToSlow();
            }


            if(DriveTrain.RobotChasis == Chassis.ChassisModes.SLOW){
                telemetry.addData("Chassis", "SLOW");
            }
            if(DriveTrain.RobotChasis == Chassis.ChassisModes.FAST){
                telemetry.addData("Chassis", "FAST");
            }


            if(gamepad2.a){

            }

            telemetry.update();
        }
    }

}