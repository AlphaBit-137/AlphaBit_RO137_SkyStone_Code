package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "BluePlate")
public class BluePlate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;


        sleep(24000);

        //Trajectory 1
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(20, 17, 2))
                        .reverse()
                        .build()
        );


        sleep(100);

        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-5, -30,1.5))
                        .build()
        );




    }
}
