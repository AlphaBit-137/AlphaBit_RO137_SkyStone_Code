
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware{

    public DcMotor motorFataStanga = null;
    public DcMotor motorSpateStanga = null;
    public DcMotor motorFataDreapta = null;
    public DcMotor motorSpateDreapta = null;
    public DcMotor motor1intk = null;
    public DcMotor motor2intk = null;
    public DcMotorEx Arm = null;
    public Servo   movebase1 = null;
    public Servo   movebase2 = null;
    public Servo   trapa1 = null;
    public Servo   trapa2 = null;
    public Servo   brat1 = null;
    public Servo   brat2 = null;

    public final static double BRAT_HOME1 = 1.0;
    public final static double BRAT_MAX1 = 0.0;
    public final static double BRAT_HOME2 = 0.5;
    public final static double BRAT_MAX2 = 0.0;

    public final static double BASE_HOME = 0.0;
    public final static double BASEE_HOME = 0.3;
    public final static double BASE_HOME2 = 0.2;
    public final static double BASE_MAX = 1.0;
    public final static double BASE_MAX2 = 0.665;
    public final static double BASE_BASE = 0.959;
    HardwareMap HardMap = null;





    NormalizedColorSensor color_sensor;

    DistanceSensor distance_sensor;




    public void init(HardwareMap PHP) {
        // Save reference to Hardware map
        HardMap = PHP;

        // Define and Initialize Motors
        distance_sensor = HardMap.get(DistanceSensor.class, "distance_sensor");
        color_sensor = HardMap.get(NormalizedColorSensor.class ,"color_sensor");
        motorFataDreapta = HardMap.get(DcMotor.class, "m3");
        motorFataStanga = HardMap.get(DcMotor.class, "m4");
        motorSpateStanga = HardMap.get(DcMotor.class, "m2");
        motorSpateDreapta = HardMap.get(DcMotor.class, "m1");


        motor1intk = HardMap.get(DcMotor.class, "m11"); // stanga
        motor2intk = HardMap.get(DcMotor.class, "m12"); // dreapta



        motorFataDreapta.setDirection(DcMotor.Direction.REVERSE);
        motorFataStanga.setDirection(DcMotor.Direction.FORWARD);
        motorSpateStanga.setDirection(DcMotor.Direction.FORWARD);
        motorSpateDreapta.setDirection(DcMotor.Direction.REVERSE);

        motor1intk.setDirection(DcMotor.Direction.FORWARD);
        motor2intk.setDirection(DcMotor.Direction.REVERSE);

        motorFataDreapta.setPower(0);
        motorFataStanga.setPower(0);
        motorSpateStanga.setPower(0);
        motorSpateDreapta.setPower(0);
        motor1intk.setPower(0);
        motor2intk.setPower(0);

        motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1intk.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2intk.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        movebase1 = HardMap.get(Servo.class, "servobaza1");
        movebase2 = HardMap.get(Servo.class, "servobaza2");

        trapa1 = HardMap.get(Servo.class, "trapa1");
        trapa2 = HardMap.get(Servo.class, "trapa2");

        brat1 = HardMap.get(Servo.class, "brat1");
        brat2 = HardMap.get(Servo.class, "brat2");

        movebase1.setPosition(BASE_HOME);
        movebase2.setPosition(BASE_HOME);

    }


    public void Init(HardwareMap PHP) {
        // Save reference to Hardware map
        HardMap = PHP;

        // Define and Initialize Motors
        distance_sensor = HardMap.get(DistanceSensor.class, "distance_sensor");
        color_sensor = HardMap.get(NormalizedColorSensor.class ,"color_sensor");
        motorFataDreapta = HardMap.get(DcMotor.class, "m3");
        motorFataStanga = HardMap.get(DcMotor.class, "m4");
        motorSpateStanga = HardMap.get(DcMotor.class, "m2");
        motorSpateDreapta = HardMap.get(DcMotor.class, "m1");
        Arm = HardMap.get(DcMotorEx.class, "Extindere");


        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor1intk = HardMap.get(DcMotor.class, "m11"); // stanga
        motor2intk = HardMap.get(DcMotor.class, "m12"); // dreapta



        motorFataDreapta.setDirection(DcMotor.Direction.REVERSE);
        motorFataStanga.setDirection(DcMotor.Direction.FORWARD);
        motorSpateStanga.setDirection(DcMotor.Direction.FORWARD);
        motorSpateDreapta.setDirection(DcMotor.Direction.REVERSE);

        motor1intk.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        motor2intk.setDirection(DcMotor.Direction.REVERSE);

        motorFataDreapta.setPower(0);
        motorFataStanga.setPower(0);
        motorSpateStanga.setPower(0);
        motorSpateDreapta.setPower(0);
        motor1intk.setPower(0);
        motor2intk.setPower(0);

        motorFataDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFataStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSpateStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSpateDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1intk.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2intk.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        movebase1 = HardMap.get(Servo.class, "servobaza1");
        movebase2 = HardMap.get(Servo.class, "servobaza2");

        trapa1 = HardMap.get(Servo.class, "trapa1");
        trapa2 = HardMap.get(Servo.class, "trapa2");

        brat1 = HardMap.get(Servo.class, "brat1");
        brat2 = HardMap.get(Servo.class, "brat2");

        movebase1.setPosition(BASE_HOME);
        movebase2.setPosition(BASE_HOME);

        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


}


