package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.structure.Gripper;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Outtake;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Linie")
public class Linie extends LinearOpMode {


     Intake intake = new Intake();
     Outtake outtake = new Outtake();
     Gripper gripper = new Gripper();

     @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

         intake.init(hardwareMap);
         outtake.init(hardwareMap);
         gripper.init(hardwareMap);




        while(!isStarted()){
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());

            telemetry.update();
            gripper.switchToOPENED();
            outtake.switchToINIT();
            outtake.update(0);
            drive.setPoseEstimate(new Pose2d(-30.00, -72.00, Math.toRadians(90)));
            drive.updatePoseEstimate();
        }

        waitForStart();

        sleep(2000);
        if(isStopRequested()) return;


        );





         drive.setPoseEstimate(new Pose2d(-30.00, 0.0, Math.toRadians(90)));
         drive.updatePoseEstimate();
            sleep(2000);


            drive.followTrajectoryGetStoneSync(
                    drive.trajectoryBuilder()
                            .setReversed(true)
                            .lineTo(new Vector2d(-30.0, 20.00))
                            .build(), outtake);


            sleep(2000);

         drive.followTrajectoryPlaceStoneSync(
                 drive.trajectoryBuilder()
                         .setReversed(true)
                         .lineTo(new Vector2d(-30.0, 72.00))
                         .build(), outtake);




     }



    }


