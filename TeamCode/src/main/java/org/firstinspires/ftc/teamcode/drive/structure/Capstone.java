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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Capstone {
    /* Public OpMode members. */

    public Servo capstone = null;

    public static double SAFE_POZ = 1.0;
    public static double PUT_POZ = 0.55;


    public CapstoneModes RobotCapstone = CapstoneModes.SAFE;

    public enum CapstoneModes {
        SAFE,
        PUT,
    }

    public Capstone() {

    }

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        capstone = hwMap.get(Servo.class, "Capstone");


    }

    public void update() {
        switch (RobotCapstone){
            case SAFE:{
                capstone.setPosition(SAFE_POZ);
                break;
            }
            case PUT:{
                capstone.setPosition(PUT_POZ);
                if(capstone.getPosition() == 0.55){
                    this.switchToSAFE();
                }
                break;
            }
        }

    }

    public void switchToSAFE(){
        RobotCapstone = RobotCapstone.SAFE;
    }
    public void switchToPUT(){
        RobotCapstone = RobotCapstone.PUT;
    }

    public boolean isSAFE(){
        if(RobotCapstone == RobotCapstone.SAFE){
            return true;
        }
        else{
            return false;
        }
    }


    public boolean isPUT(){
        if(RobotCapstone == RobotCapstone.PUT){
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


