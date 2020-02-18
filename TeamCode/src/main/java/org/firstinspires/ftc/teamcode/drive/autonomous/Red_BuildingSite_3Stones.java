package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous(name="Red_BuildingSite_3Stones", group = "RED")
public class Red_BuildingSite_3Stones extends LinearOpMode {

    public int caz = -1;

   @Override
    public void runOpMode(){

       autoGripper auto = new autoGripper();
       auto.init(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
       drive.setPoseEstimate(new Pose2d(-30.0, -72.0, Math.toRadians(90.0)));
        red_opencvSkystoneDetector rdetector = new red_opencvSkystoneDetector();
        rdetector.Init(hardwareMap);

       while (!isStarted()) {
           if (rdetector.valRight == 255 && rdetector.valMid == 255 && rdetector.valLeft == 0) {
               telemetry.addData("SkyStone", "Left");
               telemetry.update();
               caz = 1;
           } else if (rdetector.valLeft == 255 && rdetector.valMid == 255 && rdetector.valRight == 0) {
               telemetry.addData("SkyStone", "Right");
               telemetry.update();
               caz = 3;

           } else if (rdetector.valLeft == 255 && rdetector.valRight == 255 && rdetector.valMid == 0) {
               telemetry.addData("SkyStone", "Centre");
               telemetry.update();
               caz = 2;
           }
       }

        //opencvSkystoneDetector detector = new opencvSkystoneDetector();
     //  drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, true));


       waitForStart();

       rdetector.stopCamera(hardwareMap);
       auto.initpoz(hardwareMap);

       sleep(1000);

       if (isStopRequested()) return;

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
                           .setReversed(true)
                           .lineTo(new Vector2d(-2, -38))
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
                           .splineTo(new Pose2d(-55.5, -12.5, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
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
                           .splineTo(new Pose2d(-67, -4.2, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
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
                           .splineTo(new Pose2d(-45.5, -12, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
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
                           .splineTo(new Pose2d(-65, -5, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .build()
           );

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .splineTo(new Pose2d(0, -34, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(150)))
                           .lineTo(new Vector2d(70, -34), new LinearInterpolator(Math.toRadians(0), Math.toRadians(180)))
                           .build()
           );

           auto.outCollect(hardwareMap);

           drive.followTrajectorySync(
                   drive.trajectoryBuilder()
                           .setReversed(true)
                           .lineTo(new Vector2d(5, -34))
                           .build()
           );
       }



   }
}
