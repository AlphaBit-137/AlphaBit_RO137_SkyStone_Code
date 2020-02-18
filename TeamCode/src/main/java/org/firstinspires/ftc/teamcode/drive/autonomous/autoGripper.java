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

package org.firstinspires.ftc.teamcode.drive.autonomous;

import android.view.animation.LinearInterpolator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.Hardware;

public class autoGripper extends LinearOpMode {
    /* Public OpMode members. */

    public Servo grip = null;
    public DcMotorEx arm = null;
    public DcMotorEx lift = null;
    public DcMotor motorCollectDR = null;
    public DcMotor motorCollectST = null;

    public static double ARM_POWER = 0.3;
    public static double DOWN_ARM_POWER = 0.7;
    public static int ARM_INIT_POZ = -400;
    public static int DOWN_POZ = -50;

    // 550
    public static int DOWN_POZ_1 = 550;

    public static double CLOSED_POZ = 0.1;
    public static double OPENED_POZ = 0.35;

    public static double LIFT_POWER = 1.0;
    public static int LIFT_INIT_POZ = -30;
    public static int LEVEL_POZ = -5200;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        grip = hwMap.get(Servo.class, "Gripper");
        arm = hwMap.get(DcMotorEx.class, "Arm");
        lift = hwMap.get(DcMotorEx.class, "Lift");
        motorCollectDR = hwMap.get(DcMotor.class, "Right_Wing"); // stanga
        motorCollectST = hwMap.get(DcMotor.class, "Left_Wing"); // dreapta


        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCollectDR.setDirection(DcMotor.Direction.FORWARD);
        motorCollectST.setDirection(DcMotor.Direction.REVERSE);
        motorCollectDR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCollectST.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setVelocityPIDFCoefficients(6.0, 0.0, 0.0, 11.74);
        arm.setPositionPIDFCoefficients(5.0);

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void initpoz(HardwareMap ahwMap){
        motorCollectDR.setPower(0.5);
        motorCollectST.setPower(-0.5);
        arm.setTargetPosition(ARM_INIT_POZ);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
        grip.setPosition(OPENED_POZ);
    }

    public void armINIT(HardwareMap ahwMap){
        arm.setTargetPosition(ARM_INIT_POZ);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);

    }


    public void down(HardwareMap ahwMap) {
        arm.setTargetPosition(DOWN_POZ);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);

        grip.setPosition(CLOSED_POZ);
    }

    public void armDown(HardwareMap ahwMap) {
        arm.setTargetPosition(DOWN_POZ);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);

    }

    public void startCollect(HardwareMap ahwMap) {
        motorCollectDR.setPower(0.6);
        motorCollectST.setPower(-0.6);
    }
    public void stopCollect(HardwareMap ahwMap){
        motorCollectDR.setPower(0);
        motorCollectST.setPower(0);
    }
    public void outCollect(HardwareMap ahwMap){
        motorCollectDR.setPower(-0.3);
        motorCollectST.setPower(0.3);
    }


    public void servoIN(){
        grip.setPosition(CLOSED_POZ);
    }

    public void outTake(HardwareMap ahwMap){

        grip.setPosition(CLOSED_POZ);

        lift.setTargetPosition(LEVEL_POZ);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(LIFT_POWER);

        arm.setTargetPosition(ARM_INIT_POZ);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);


     /*   if(lift.getCurrentPosition() > -4000) {
            arm.setTargetPosition(DOWN_POZ_1);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm.setPower(ARM_POWER);
        }
        if(arm.getCurrentPosition()> 400){
            grip.setPosition(OPENED_POZ);
        }*/

     if(lift.getCurrentPosition() < -4500) {
         arm.setTargetPosition(DOWN_POZ_1);
         arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
         arm.setPower(ARM_POWER);

     }

    }


    @Override
    public void runOpMode() {}

    public void gripDOWN(HardwareMap ahwMap) {
        grip.setPosition(OPENED_POZ);
    }

    public void outakeDOWN(HardwareMap ahwMap) {


        if (arm.getCurrentPosition() != ARM_INIT_POZ && arm.getCurrentPosition() > 200) {
            arm.setTargetPosition(ARM_INIT_POZ);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            arm.setPower(DOWN_ARM_POWER);

            lift.setTargetPosition(LIFT_INIT_POZ);
            lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift.setPower(LIFT_POWER);
        }
    }





}

