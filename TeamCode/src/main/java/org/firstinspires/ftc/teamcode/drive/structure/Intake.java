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

package org.firstinspires.ftc.teamcode.drive.structure;

import android.provider.DocumentsContract;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

public class Intake {
    /* Public OpMode members. */

    public DcMotorEx leftWing = null;
    public DcMotorEx rightWing = null;

    public static double NULL_POWER = 0.0;
    public static double IN_POWER = 0.5;
    public static double OUT_POWER = -0.5;

    public IntakeModes RobotIntake = IntakeModes.STOP;

    public enum IntakeModes {
        IN,
        OUT,
        STOP,
    }

    public Intake() {

    }

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        leftWing = hwMap.get(DcMotorEx.class, "Left_Wing");
        leftWing.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftWing.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftWing.setPower(0);

        rightWing = hwMap.get(DcMotorEx.class, "Right_Wing");
        rightWing.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightWing.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightWing.setPower(0);

    }

    public void update() {
        switch (RobotIntake){
            case STOP:{
                leftWing.setPower(0);
                rightWing.setPower(0);
                break;
            }
            case IN:{
                leftWing.setPower(-0.5);
                rightWing.setPower(0.5);
                break;
            }
            case OUT:{
                leftWing.setPower(0.2);
                rightWing.setPower(-0.2);
                break;
            }
        }

    }

    public void switchToIN(){
        RobotIntake = IntakeModes.IN;
    }

    public void switchToOUT(){
        RobotIntake = IntakeModes.OUT;
    }

    public void switchToSTOP(){RobotIntake = IntakeModes.STOP;}

    public boolean isIN(){
        if(RobotIntake == IntakeModes.IN){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isOUT(){
        if(RobotIntake == IntakeModes.OUT){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isSTOP(){
        if(RobotIntake == IntakeModes.STOP){
            return true;
        }
        else{
            return false;
        }
    }






    /* EX 2 -> 0 Gheara stanga

        EX 1 -> 5 Gheara dreapta


        EX 1 -> 0  -> Right Wing
        EX 1 -> 1  -> Left Wing


        EX 1 -> 2 -> Lift


         Gherute
     */





}

