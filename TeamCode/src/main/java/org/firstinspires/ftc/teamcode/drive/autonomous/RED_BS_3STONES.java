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
@Autonomous(name="RED_BS_3STONES", group = "RED")
public class RED_BS_3STONES extends LinearOpMode {


    public int caz = -1;

   @Override
    public void runOpMode(){

       autoGripper auto = new autoGripper();
       auto.init(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
       drive.setPoseEstimate(new Pose2d(-30.0, -72.0, Math.toRadians(90.0)));
        opencvSkystoneDetector sdetector = new opencvSkystoneDetector();
        sdetector.Init(hardwareMap);

       while (!isStarted()) {
           if (sdetector.valRight == 255 && sdetector.valMid == 255 && sdetector.valLeft == 0) {
               telemetry.addData("SkyStone", "Left");
               telemetry.update();
               caz = 1;
           } else if (sdetector.valLeft == 255 && sdetector.valMid == 255 && sdetector.valRight == 0) {
               telemetry.addData("SkyStone", "Right");
               telemetry.update();
               caz = 3;

           } else if (sdetector.valLeft == 255 && sdetector.valRight == 255 && sdetector.valMid == 0) {
               telemetry.addData("SkyStone", "Center");
               telemetry.update();
               caz = 2;
           }
       }
        //opencvSkystoneDetector detector = new opencvSkystoneDetector();
     //  drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, true));



       waitForStart();
       sdetector.stopCamera(hardwareMap);
       auto.initpoz(hardwareMap);
       sleep(1000);

       if (isStopRequested()) return;


       // LinearInterpolator interp = new LinearInterpolator(Math.toRadians(90), Math.toRadians(50));
        //CUB 3 - Colectare cub





       //CAZ STANGA
       if(caz == 1) {
           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .splineTo(new Pose2d(-37, -30, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(90), Math.toRadians(120)))
                           .build()
           );

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(10, -48, Math.toRadians(180)))
                           .lineTo(new Vector2d(45, -48), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .lineTo(new Vector2d(0, -48), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.startCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .splineTo(new Pose2d(-71, -5, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, -38, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(65, -38), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .lineTo(new Vector2d(0, -38), new LinearInterpolator(Math.toRadians(0),Math.toRadians(180)))
                           .build()
           );

           //NU MERGE WTF

           /*auto.startCollect(hardwareMap);

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .splineTo(new Pose2d(-20, -33, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, -33, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(65, -33), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .lineTo(new Vector2d(5, -33))
                           .build()
           );*/


       }






       //CAZ CENTRU

       else if(caz == 2){
           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .splineTo(new Pose2d(-27, -33, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(90), Math.toRadians(120)))
                           .build()
           );

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(10, -52, Math.toRadians(180)))
                           .lineTo(new Vector2d(45, -52), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .lineTo(new Vector2d(0, -52), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.startCollect(hardwareMap);

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .splineTo(new Pose2d(-58.9, -13, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );
           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, -42, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(50, -42), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .lineTo(new Vector2d(0, -42), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.startCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .splineTo(new Pose2d(-70.9, -5, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, -31, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(65, -31), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .lineTo(new Vector2d(5, -31))
                           .build()
           );
       }







       // CAZ DREAPTA
       else if(caz == 3 ){
           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .splineTo(new Pose2d(-20, -33, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(90), Math.toRadians(120)))
                           .build()
           );


           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(10, -52, Math.toRadians(180)))
                           .lineTo(new Vector2d(45, -52), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .lineTo(new Vector2d(0, -52), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.startCollect(hardwareMap);

           drive.followTrajectorySync(

                   drive.trajectoryBuilder()
                           .splineTo(new Pose2d(-48, -14, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, -47, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(50, -47), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(false)
                           .lineTo(new Vector2d(0, -47), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.startCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .splineTo(new Pose2d(-69, -6, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, -37, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(65, -37), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .lineTo(new Vector2d(5, -37))
                           .build()
           );
       }



   }
}
