package org.firstinspires.ftc.teamcode.drive.autonomous;

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
@Autonomous(name="CUB3", group = "Blue_Quarry_Incomplete")
public class Ichim_Blue extends LinearOpMode {

    OpenCvWebcam webcam;
   @Override
    public void runOpMode() throws InterruptedException{

       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
       webcam.openCameraDevice();
       webcam.setPipeline(new opencvSkystoneDetector.StageSwitchingPipeline());
       webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

       autoGripper auto = new autoGripper();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        //opencvSkystoneDetector detector = new opencvSkystoneDetector();
     //  drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, true));
       auto.init(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-30.0, -72.0, Math.toRadians(90.0)));
         auto.startCollect(hardwareMap);

        waitForStart();


        if (isStopRequested()) return;


       // LinearInterpolator interp = new LinearInterpolator(Math.toRadians(90), Math.toRadians(50));
        //CUB 3 - Colectare cub
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(-20, -33, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(90.00), Math.toRadians(120)))
                        .build()
        );
       auto.stopCollect(hardwareMap);
       auto.down(hardwareMap);


        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(25, -49, Math.toRadians(180)))

                        .build()
        );

        sleep(100);


        auto.outTake(hardwareMap);
                drive.followTrajectorySync(

                        drive.trajectoryBuilder()
                                .setReversed(true)
                                .splineTo(new Pose2d(55, -35, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(270), Math.toRadians(180)))
                                .build()
                );

            auto.outTake(hardwareMap);



       drive.followTrajectorySync(
               drive.trajectoryBuilder()
                       .setReversed(false)
                       .splineTo(new Pose2d(30, -47, Math.toRadians(180)))
                       .build()
       );
       auto.gripDOWN(hardwareMap);
       sleep(500);
       auto.outakeDOWN(hardwareMap);
       sleep(1000);


       drive.followTrajectorySync(
               drive.trajectoryBuilder()
                       .setReversed(false)
                        .lineTo(new Vector2d(-10, -47))
                       .splineTo(new Pose2d(-63, -33, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180), Math.toRadians(120)))
                       .build()
       );
        auto.startCollect(hardwareMap);
       auto.down(hardwareMap);



   }
}
