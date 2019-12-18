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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.security.KeyStore;

public class Chassis
{
    /* Public OpMode members. */

    public DcMotor  RightFront   = null;    //Config: 0
    public DcMotor  RightBack  = null;      //Config: 1
    public DcMotor  LeftBack = null;        //Config: 2
    public DcMotor  LeftFront = null;       //Config: 3

    boolean IS_DISABLED = false;

    public ChassisModes  RobotChasis = ChassisModes.FAST;


    private enum ChassisModes{
            FAST,
            SLOW,
    }

    public double RightFrontPower = 0;
    public double RightBackPower = 0;
    public double LeftBackPower = 0;
    public double LeftFrontPower = 0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Chassis(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LeftBack = hwMap.get(DcMotor.class, "Left_Back");
        RightFront = hwMap.get(DcMotor.class, "Right_Front");
        LeftFront = hwMap.get(DcMotor.class, "Left_Front");
        RightBack = hwMap.get(DcMotor.class, "Right_Back");

        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        LeftBack.setPower(0);
        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void update() {
        if (IS_DISABLED)
            return;
        switch (RobotChasis){
            case FAST:{
                Range.clip(LeftFrontPower, -1, 1);
                Range.clip(LeftFrontPower, -1, 1);
                Range.clip(RightBackPower, -1, 1);
                Range.clip(RightFrontPower, -1, 1);
                MotorSetter(LeftFrontPower, LeftBackPower, RightBackPower, RightFrontPower);
                break;
            }
            case SLOW:{
                Range.clip(LeftFrontPower, -0.2, 0.2);
                Range.clip(LeftFrontPower, -0.2, 0.2);
                Range.clip(RightBackPower, -0.2, 0.2);
                Range.clip(RightFrontPower, -0.2, 0.2);
                MotorSetter(LeftFrontPower, LeftBackPower, RightBackPower, RightFrontPower);
                break;

            }
        }
    }

    public void switchToFast(){
        RobotChasis=ChassisModes.FAST;
    }
    public void switchToSlow(){
        RobotChasis=ChassisModes.SLOW;
    }


    public void MotorSetter(double x1, double x2, double x3, double x4){
        LeftFront.setPower(x1);
        LeftBack.setPower(x1);
        RightBack.setPower(x1);
        RightFront.setPower(x1);

    }



 }

