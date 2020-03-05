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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    /* Public OpMode members. */

    public DcMotorEx arm = null;
    public static int RESET_POZ = -300;
    public static int GET_POZ = -50;
    public static double ARM_POWER = 0.6;
    public static double RESET_ARM__POWER = 1.0;

    public ArmModes RobotArm = ArmModes.INIT;

    public enum ArmModes {
        INIT,
        GET,
        SCORE,
    }

    public Arm() {

    }

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        arm = hwMap.get(DcMotorEx.class, "Arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setVelocityPIDFCoefficients(6.0, 0.0, 0.0, 11.74);
        arm.setPositionPIDFCoefficients(5.0);
    }

    public void update(int armEncoderCount) {
        switch (RobotArm){
            case INIT:{
                arm.setTargetPosition(RESET_POZ);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(RESET_ARM__POWER);
                break;
            }
            case GET:{
                arm.setTargetPosition(GET_POZ);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(ARM_POWER);
                break;
            }
            case SCORE:{
                arm.setTargetPosition(armEncoderCount);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setPower(ARM_POWER);
            }
        }

    }

    public void switchToINIT(){
        RobotArm = ArmModes.INIT;
    }

    public void switchToGET(){
        RobotArm = ArmModes.GET;
    }

    public void switchToScore(){RobotArm = ArmModes.SCORE;}

    public int getArmEncoder(){
        return arm.getCurrentPosition();
    }


    public boolean isINIT(){
        if(RobotArm == ArmModes.INIT){
            return true;
        }else{
            return false;
        }
    }

    public boolean isGET(){
        if(RobotArm == ArmModes.GET){
            return true;
        }else{
            return false;
        }
    }

    public boolean isSCORE(){
        if(RobotArm == ArmModes.SCORE){
            return true;
        }else{
            return false;
        }
    }



}

