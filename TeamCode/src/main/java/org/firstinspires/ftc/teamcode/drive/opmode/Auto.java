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
@Autonomous(group = "Auto")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);


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
                        .splineTo(new Pose2d(25,-22.5, 67.5))
                        .build()
        );

        drive.turnSync(Math.toRadians(15));




        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(22.5, 3.5))
                        .build()
        );

        sleep(10);



        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(22.5, 10))
                        .build()

        );

        drive.turnSync(Math.toRadians(90));

        drive.followTrajectorySync(

                drive.trajectoryBuilder()

                        .splineTo(new Pose2d(40, 6, 0))
                        .build()
        );


        drive.turnSync(Math.toRadians(-180));

        drive.followTrajectorySync(

                drive.trajectoryBuilder()

                        .splineTo(new Pose2d(30,-22, 67.5))
                        .build()
        );

        drive.turnSync(Math.toRadians(18));



        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(22.5, 6.5))
                        .build()
        );

        sleep(10);



        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(22.5, 2))
                        .splineTo(new Pose2d(50, 6,135))
                        .build()

        );


        drive.turnSync(Math.toRadians(-50));


        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(43, 8.5))
                        .build()

        );











    }
}
