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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    /* Public OpMode members. */

    public DcMotorEx lift = null;


    public LiftModes RobotLift = LiftModes.INIT;

    public enum LiftModes {
        INIT,
        LEVEL,
        SCORE,
        FREE,
    }

    public Lift() {

    }

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        lift = hwMap.get(DcMotorEx.class, "Lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void update(int scorePosition, double freePower) {
        switch (RobotLift){
            case INIT:{
                lift.setTargetPosition(-30);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lift.setPower(0.8);
                break;
            }
            case LEVEL:{
                lift.setTargetPosition(-5000);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lift.setPower(1.0);
                break;
            }
            case SCORE:{
                lift.setTargetPosition(scorePosition);
                lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                lift.setPower(1.0);
                break;
            }
            case FREE:{
                lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                lift.setPower(freePower);
                break;

            }

        }

    }

    public void switchToINIT(){
        RobotLift = LiftModes.INIT;
    }
    public void switchToLEVEL(){
        RobotLift = LiftModes.LEVEL;
    }
    public void switchToSCORE(){ RobotLift = LiftModes.SCORE; }
    public void switchToFREE(){ RobotLift = LiftModes.FREE; }

    public int getLiftEncoder(){
        return lift.getCurrentPosition();
    }

    public boolean isINIT(){
        if(RobotLift == LiftModes.INIT){
            return true;
        }else{
            return false;
        }
    }

    public boolean isLEVEL(){
        if(RobotLift == LiftModes.LEVEL){
            return true;
        }else{
            return false;
        }
    }

    public boolean isSCORE(){
        if(RobotLift == LiftModes.SCORE){
            return true;
        }else{
            return false;
        }
    }

    public boolean isFREE(){
        if(RobotLift == LiftModes.FREE){
            return true;
        }else{
            return false;
        }
    }


    
    
    
    

}

