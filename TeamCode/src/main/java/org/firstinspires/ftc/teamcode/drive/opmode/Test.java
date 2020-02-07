package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous(group = "TestGenerat")

public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-37, -72, Math.toRadians(90)));


        drive.setPoseEstimate(new Pose2d(-37.0,72.0,Math.toRadians(270)));


        waitForStart();

        if (isStopRequested()) return;

        //Trajectory 1
        drive.followTrajectorySync(

                drive.trajectoryBuilder()

                        .splineTo(new Pose2d(0, -48, Math.toRadians(0)))
                        .build()
        );





    }
}

