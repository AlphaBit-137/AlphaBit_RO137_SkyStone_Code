package org.firstinspires.ftc.teamcode.drive.autonomous;

import android.graphics.Interpolator;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.opmode.Hardware;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="AutoTest", group = "BLUE")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        autoGripper auto = new autoGripper();
        auto.init(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-42.0, 72.0, Math.toRadians(270.0)));

        waitForStart();

        auto.initpoz(hardwareMap);
        sleep(1000);

        drive.update();


        }



    }

