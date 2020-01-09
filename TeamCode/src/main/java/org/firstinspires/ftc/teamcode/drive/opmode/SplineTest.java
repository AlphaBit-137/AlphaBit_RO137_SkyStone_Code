package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(15, 15, 90))
                        .splineTo(new Pose2d(0, 30, 135))
                        .splineTo(new Pose2d(-15, 15, 225))
                        .splineTo(new Pose2d(0,0, 360))
                        .build()
        );

        sleep(2000);

      /*drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 0, 180))
                        .build()
        );

    */


    }
}
