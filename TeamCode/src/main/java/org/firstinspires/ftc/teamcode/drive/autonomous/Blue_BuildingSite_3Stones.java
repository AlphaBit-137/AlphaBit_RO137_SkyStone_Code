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
@Autonomous(name="Blue_BuildingSite_3Stones", group = "BLUE")
public class Blue_BuildingSite_3Stones extends LinearOpMode {


    public int caz = -1;

    @Override
    public void runOpMode(){

        autoGripper auto = new autoGripper();
        auto.init(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-42.0, 72.0, Math.toRadians(270.0)));
        blue_opencvSkystoneDetector bdetector = new blue_opencvSkystoneDetector();
        bdetector.Init(hardwareMap);

        while (!isStarted()) {
            if (bdetector.valRight == 255 && bdetector.valMid == 255 && bdetector.valLeft == 0) {
                telemetry.addData("SkyStone", "Left");
                telemetry.update();
                caz = 1;
            } else if (bdetector.valLeft == 255 && bdetector.valMid == 255 && bdetector.valRight == 0) {
                telemetry.addData("SkyStone", "Right");
                telemetry.update();
                caz = 3;

            } else if (bdetector.valLeft == 255 && bdetector.valRight == 255 && bdetector.valMid == 0) {
                telemetry.addData("SkyStone", "Centre");
                telemetry.update();
                caz = 2;
            }
        }
        //opencvSkystoneDetector detector = new opencvSkystoneDetector();
        //  drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, true));

        waitForStart();

        bdetector.stopCamera(hardwareMap);
        auto.initpoz(hardwareMap);
        sleep(1000);

        if (isStopRequested()) return;

        //CAZ DREAPTA
        if(caz == 3) {
            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-33.2, 26, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(270), Math.toRadians(200)))
                            .build()
            );

            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(0, 44, Math.toRadians(180)))
                            .lineTo(new Vector2d(50, 44), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .lineTo(new Vector2d(-10, 44), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.startCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-69, 21, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(210)))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(15, 38, Math.toRadians(180)))
                            .lineTo(new Vector2d(50, 38), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(-6,38,Math.toRadians(0)), new SplineInterpolator(Math.toRadians(0),Math.toRadians(0)))
                            .build()
            );

            //NU MERGE WTF

           /*auto.startCollect(hardwareMap);

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .splineTo(new Pose2d(-20, 33, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, 33, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(65, 33), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .lineTo(new Vector2d(5, 33))
                           .build()
           );*/


        }






        //CAZ CENTRU

        else if(caz == 2){
            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-22, 27, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(270), Math.toRadians(210)))
                            .build()
            );

            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(-10, 51, Math.toRadians(180)))
                            .lineTo(new Vector2d(45, 51), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .lineTo(new Vector2d(0, 51), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.startCollect(hardwareMap);

            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-53, 13, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(210)))
                            .build()
            );
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(0, 43, Math.toRadians(180)))
                            .lineTo(new Vector2d(60, 43), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .lineTo(new Vector2d(-15, 43), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.startCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-57, 12, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(230)))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(0, 40, Math.toRadians(180)))
                            .lineTo(new Vector2d(65, 40), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(-5,40,Math.toRadians(0)), new SplineInterpolator(Math.toRadians(0),Math.toRadians(0)))
                            .build()
            );
        }







        // CAZ STANGA
        else if(caz == 1 ){
            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(-15.5, 31, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(270), Math.toRadians(230)))
                            .build()
            );


            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(10, 46, Math.toRadians(180)))
                            .lineTo(new Vector2d(45, 46), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .lineTo(new Vector2d(0, 46), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.startCollect(hardwareMap);

            drive.followTrajectorySync(

                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-55, 14, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(210)))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(0, 43, Math.toRadians(180)))
                            .lineTo(new Vector2d(50, 43), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .lineTo(new Vector2d(0, 43), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.startCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-71, 11, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(210)))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(0, 38, Math.toRadians(180)))
                            .lineTo(new Vector2d(65, 38), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                            .build()
            );

            auto.outCollect(hardwareMap);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(-3,38,Math.toRadians(0)), new SplineInterpolator(Math.toRadians(0),Math.toRadians(0)))
                            .build()
            );
        }



    }
}
