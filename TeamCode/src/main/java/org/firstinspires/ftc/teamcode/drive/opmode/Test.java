package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "BluePlate")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-37.0,72.0,Math.toRadians(270)));

        waitForStart();

        if (isStopRequested()) return;

        //Trajectory 1
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-30.0,25.0,Math.toRadians(270)))
                        .build()
        );

        drive.turnSync(Math.toRadians(180));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-20.0,45.0,Math.toRadians(0)))
                        .build()
        );


        /*drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-60.0,25.0,Math.toRadians(270)))
                        .build()
        );

        drive.turnSync(Math.toRadians(90));

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0.0,38.0,Math.toRadians(180)))
                        .lineTo(new Vector2d(12.0,38.0))
                        .build()
        );*/
    }
}

