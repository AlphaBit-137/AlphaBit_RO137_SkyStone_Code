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

import com.fasterxml.jackson.core.io.DataOutputAsStream;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    /* Public OpMode members. */
    Gripper gripper;
    Lift lift;
    Arm arm;
    int stoneNumber  = 0;

    public OuttakeModes RobotOuttake = OuttakeModes.RESET;

    public enum OuttakeModes {
        STONE_1,
        STONE_2,
        STONE_3,
        STONE_4,
        STONE_5,
        STONE_6,
        STONE_7,
        STONE_8,
        STONE_9,
        GETSTONE,
        RESET,
        INIT,
    }

    public Outtake() {
        this.gripper = new Gripper();
        this.lift = new Lift();
        this.arm = new Arm();

    }
    /* local OpMode members. */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        gripper.init(ahwMap);
        lift.init(ahwMap);
        arm.init(ahwMap);
    }
    public void update(double freePower){
        int armScorePosition = 0;
        int liftScorePosition = 0;
        switch (RobotOuttake){
            case INIT:{
                lift.switchToINIT();
                arm.switchToINIT();
                gripper.switchToOPENED();
                break;

            }

            case STONE_1: {
                stoneNumber = 1;
                armScorePosition = 550;
                liftScorePosition = -2500;

                if(arm.isGET() && lift.isINIT()){
                    lift.switchToLEVEL();
                }
                if(lift.isLEVEL() && lift.getLiftEncoder() < -3500){
                    arm.switchToScore();
                }
                if(arm.isSCORE() && arm.getArmEncoder() > 350){
                    lift.switchToSCORE();
                }
                break;
            }

            case STONE_2: {
                stoneNumber = 2;
                armScorePosition = 550;
                liftScorePosition = -4500;

                if(arm.isGET() && lift.isINIT()){
                    lift.switchToLEVEL();
                }
                if(lift.isLEVEL() && lift.getLiftEncoder() < -4500){
                    arm.switchToScore();
                }
                if(arm.isSCORE() && arm.getArmEncoder() > 350){
                    lift.switchToSCORE();
                }
                break;
            }

            case STONE_3: {
                stoneNumber = 3;
                armScorePosition = 550;
                liftScorePosition = -6000;

                if(arm.isGET() && lift.isINIT()){
                    lift.switchToSCORE();
                }
                if(lift.isSCORE() && lift.getLiftEncoder() < -4500){
                    arm.switchToScore();
                }
                break;
            }

            case STONE_4: {
                stoneNumber = 4;
                armScorePosition = 700;
                liftScorePosition = -4000;

                if(arm.isGET() && lift.isINIT()){
                    lift.switchToLEVEL();
                }
                if(lift.isLEVEL() && lift.getLiftEncoder() < -4500){
                    arm.switchToScore();
                }
                if(arm.isSCORE() && arm.getArmEncoder() > 350){
                    lift.switchToSCORE();
                }
                break;
            }





            case GETSTONE: {
                    arm.switchToGET();
                if(arm.isGET() && arm.getArmEncoder()< -100){
                    gripper.switchToCLOSED();
                }

                break;
            }

            case RESET:{
                if(stoneNumber == 1) {
                    armScorePosition = 550;
                    liftScorePosition = -2500;
                }
                if(stoneNumber == 2){
                    armScorePosition = 550;
                    liftScorePosition = -4500;

                }
                if(stoneNumber == 3){
                    armScorePosition = 550;
                    liftScorePosition = -6000;
                }
                if(stoneNumber == 4) {
                    armScorePosition = 700;
                    liftScorePosition = -4000;
                }
                if(stoneNumber == 5){
                    armScorePosition = 550;
                    liftScorePosition = -4500;

                }
                if(stoneNumber == 6){
                    armScorePosition = 550;
                    liftScorePosition = -6000;
                }
                if(stoneNumber == 7) {
                    armScorePosition = 550;
                    liftScorePosition = -2500;
                }
                if(stoneNumber == 8){
                    armScorePosition = 550;
                    liftScorePosition = -4500;

                }
                if(stoneNumber == 9){
                    armScorePosition = 550;
                    liftScorePosition = -6000;
                }




                if(arm.isSCORE()){
                    lift.switchToLEVEL();
                }
                if(lift.isLEVEL() && lift.getLiftEncoder() < -3500){
                    arm.switchToINIT();
                }
                if(arm.isINIT() && arm.getArmEncoder() <100){
                    lift.switchToINIT();
                    gripper.switchToOPENED();

                }
                break;
            }



        }

        gripper.update();
        lift.update(liftScorePosition,freePower);
        arm.update(armScorePosition);

    }


    public void switchToINIT(){
        RobotOuttake = OuttakeModes.INIT;
    }
    public void switchToSTONE_1(){
        RobotOuttake = OuttakeModes.STONE_1;
    }
    public void switchToSTONE_2(){
        RobotOuttake = OuttakeModes.STONE_2;
    }
    public void switchToSTONE_3(){
        RobotOuttake = OuttakeModes.STONE_3;
    }
    public void switchToSTONE_4(){
        RobotOuttake = OuttakeModes.STONE_4;
    }
    public void switchToSTONE_5(){
        RobotOuttake = OuttakeModes.STONE_5;
    }
    public void switchToSTONE_6(){
        RobotOuttake = OuttakeModes.STONE_6;
    }
    public void switchToSTONE_7(){
        RobotOuttake = OuttakeModes.STONE_7;
    }
    public void switchToSTONE_8(){
        RobotOuttake = OuttakeModes.STONE_8;
    }
    public void switchToSTONE_9(){
        RobotOuttake = OuttakeModes.STONE_9;
    }
    public void switchToRESET(){ RobotOuttake = OuttakeModes.RESET; }
    public void switchToGETSTONE(){ RobotOuttake = OuttakeModes.GETSTONE;}

    public boolean isINIT(){
        if(RobotOuttake == OuttakeModes.INIT){
            return true;
        }else{
            return false;
        }
    }

    public boolean isSTONE_1(){
        if(RobotOuttake == OuttakeModes.STONE_1){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_2(){
        if(RobotOuttake == OuttakeModes.STONE_2){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_3(){
        if(RobotOuttake == OuttakeModes.STONE_3){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_4(){
        if(RobotOuttake == OuttakeModes.STONE_4){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_5(){
        if(RobotOuttake == OuttakeModes.STONE_5){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_6(){
        if(RobotOuttake == OuttakeModes.STONE_6){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_7(){
        if(RobotOuttake == OuttakeModes.STONE_7){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_8(){
        if(RobotOuttake == OuttakeModes.STONE_8){
            return true;
        }else{
            return false;
        }
    }
    public boolean isSTONE_9(){
        if(RobotOuttake == OuttakeModes.STONE_9){
            return true;
        }else{
            return false;
        }
    }

    public boolean isRESET(){
        if(RobotOuttake == OuttakeModes.RESET){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isGETSTONE(){
        if(RobotOuttake == OuttakeModes.GETSTONE){
            return true;
        }
        else{
            return false;
        }
    }









}

