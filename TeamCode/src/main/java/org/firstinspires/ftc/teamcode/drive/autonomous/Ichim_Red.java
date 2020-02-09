package org.firstinspires.ftc.teamcode.drive.autonomous;

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
@Autonomous(group = "Ichim_Red")
public class Ichim_Red extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        // drive.setPoseEstimate(new Pose2d(30.00, 30.00, 0.00));

        waitForStart();

        if (isStopRequested()) return;

        //Trajectory 1
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30, -4.3, 0))
                        .splineTo(new Pose2d(45, -6, 0))
                        .build()
        );

        drive.turnSync(Math.toRadians(-180));


        //Trajectory 2
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(25,-22.5, 67.87))
                        .build()
        );

//        drive.turnSync(Math.toRadians(15));



        //Trajectory_3
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(22.5, 9))
                        .build()
        );

        sleep(10);


        //Trajectory_4
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(22.5, 10))
                        .build()

        );

        drive.turnSync(Math.toRadians(90));

        //Trajectory_5
        drive.followTrajectorySync(

                drive.trajectoryBuilder()

                        .splineTo(new Pose2d(50, 8, 0))
                        .build()
        );


        drive.turnSync(Math.toRadians(-180));
        //Trajectory 6
        drive.followTrajectorySync(

                drive.trajectoryBuilder()

                        .splineTo(new Pose2d(35,-35, 62))
                        //65
                        .build()
        );

        //  drive.turnSync(Math.toRadians(18));

///de aici trebuie sa modific
        //Trajectory_7
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(0, 5))
                        .build()
        );

        sleep(10);


        //Trajectory_8
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(30, -8))
                        .splineTo(new Pose2d(60, -4,135))
                        .setReversed(false)
                        .splineTo(new Pose2d( 40, 27, 135 ))
                        .splineTo(new Pose2d(28, -20,5.05))
                        .build()

        );
        drive.turnSync(Math.toRadians(9));

        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .strafeLeft(5)
                        .lineTo(new Vector2d(28,-60))
                        .reverse()
                        .lineTo(new Vector2d(28,-30))
                        .build()

        );






    }
}
