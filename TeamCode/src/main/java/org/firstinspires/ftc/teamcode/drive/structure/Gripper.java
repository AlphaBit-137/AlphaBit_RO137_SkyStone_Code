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

public class Gripper {
    /* Public OpMode members. */

    public Servo grip = null;

    public static double CLOSED_POZ = 0.15;
    public static double OPENED_POZ = 0.35;

    public GripperModes RobotGripper = GripperModes.OPENED;

    public enum GripperModes {
        OPENED,
        CLOSED,
    }

    public Gripper() {

    }

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        grip = hwMap.get(Servo.class, "Gripper");
    }

    public void update() {
        switch (RobotGripper){
            case OPENED:{
                grip.setPosition(OPENED_POZ);
                break;
            }
            case CLOSED:{
                grip.setPosition(CLOSED_POZ);
                break;
            }
        }

    }

    public void switchToOPENED(){
        RobotGripper = GripperModes.OPENED;
    }

    public void switchToCLOSED(){
        RobotGripper = GripperModes.CLOSED;
    }

    public boolean isCLOSED(){
        if (RobotGripper == GripperModes.CLOSED){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isOPENED(){
        if (RobotGripper == GripperModes.OPENED){
            return true;
        }
        else{
            return false;
        }
    }




}

