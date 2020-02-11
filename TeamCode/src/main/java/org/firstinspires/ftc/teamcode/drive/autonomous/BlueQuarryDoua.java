package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "Blue_Quarry_Incomplete")
public class BlueQuarryDoua extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        //Trajectory 1
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30, 4.3, 0))
                        .splineTo(new Pose2d(45,6, 0))
                        .build()
        );

        drive.turnSync(Math.toRadians(180));


        //Trajectory 2
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(25,22.5, 65))
                        .build()
        );

        drive.turnSync(Math.toRadians(-44));




        //Trajectory_3
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(25, 50))
                        .build()
        );

        sleep(10);


        //Trajectory_4
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(22.5, -8.5))
                        .build()

        );


        drive.turnSync(Math.toRadians(-88));


        //Trajectory_5
        drive.followTrajectorySync(

                drive.trajectoryBuilder()

                        .splineTo(new Pose2d(45, -6, 0))
                        .build()
        );


        drive.turnSync(Math.toRadians(178));

        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30,22.5, 70.3))
                        .build()
        );


        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(30, 50))
                        .build()

        );

        LinearInterpolator interp = new LinearInterpolator(Math.toRadians(30), Math.toRadians(60));

        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .lineTo( new Vector2d(50, 50), interp)
                        .build()

        );

        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .reverse()
                        .lineTo(new Vector2d(30, 35))
                        .build()

        );

    }

}
