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

public class Claws {
    /* Public OpMode members. */

    public Servo leftClaw = null;
    public Servo rightClaw = null;


    private double leftClawOpen = 0.0; //
    private double rightClawOpen = 1.0;
    private double leftClawClosed = 1.0;
    private double rightClawClosed = 0.0;


    public ClawModes RobotClaw= ClawModes.OPENED;

    public enum ClawModes {
        OPENED,
        CLOSED,
    }

    public Claws() {

    }

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftClaw = hwMap.get(Servo.class, "Left_Claw");
        rightClaw = hwMap.get(Servo.class, "Right_Claw");
    }

    public void update() {
        switch (RobotClaw){
            case OPENED:{
                leftClaw.setPosition(leftClawOpen);
                rightClaw.setPosition(rightClawOpen);

                break;
            }
            case CLOSED:{
                leftClaw.setPosition(leftClawClosed);
                rightClaw.setPosition(rightClawClosed);
                break;
            }
        }

    }

    public void switchToOPENED(){
        RobotClaw = ClawModes.OPENED;
    }

    public void switchToCLOSED(){
        RobotClaw = ClawModes.CLOSED;
    }

    public boolean isCLOSED(){
        if (RobotClaw == ClawModes.CLOSED){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isOPENED(){
        if (RobotClaw == ClawModes.OPENED){
            return true;
        }
        else{
            return false;
        }
    }




}

