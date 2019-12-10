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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OPMode_Principal", group="Linear Opmode")

public class OPMode_Principal extends LinearOpMode {

    // Declaram obiectul robot cu clasa hardware si timpul de rulare

    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();


    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        runtime.reset();
        waitForStart();

        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod

        while (opModeIsActive()) {

            //Initiallizam variabilele
            double Front, Turn, Sum, Diff, Side, Drive1, Drive2, Drive3, Drive4;

            //Primirea datelor de la joystick-uri
            Front = gamepad1.left_stick_y;
            Turn = gamepad1.right_stick_x;
            Side = gamepad1.left_stick_x;

            //Calcularea puterii redate motoarelor
            Sum = Range.clip(Front + Side, -1.0, 1.0);
            Diff = Range.clip(Front - Side, -1.0, 1.0);

            Drive1 = Range.clip(Sum - 2*Turn, -1.0, 1.0);
            Drive2 = Range.clip(Sum + 2*Turn, -1.0, 1.0);
            Drive3 = Range.clip(Diff - 2*Turn, -1.0, 1.0);
            Drive4 = Range.clip(Diff + 2*Turn, -1.0, 1.0);

            MS(Drive1, Drive2, Drive3, Drive4);

            telemetry.addData("Motors", "BackLeft (%.2f), FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f)", Drive1, Drive2, Drive3, Drive4);
            telemetry.addData("Informatie:", "Atentie! Programul a fost sting.");
            telemetry.update();
        }
    }

    void MS(double x1, double x2, double x3, double x4){
        robot.BackLeftMotor.setPower(x1);
        robot.FrontRightMotor.setPower(x2);
        robot.FrontLeftMotor.setPower(x3);
        robot.BackRightMotor.setPower(x4);
    }

}