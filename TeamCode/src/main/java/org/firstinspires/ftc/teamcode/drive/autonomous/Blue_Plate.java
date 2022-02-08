package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous(name="Blue_Plate", group = "BLUE")
public class Blue_Plate extends LinearOpMode {

    @Override
    public void runOpMode(){

        autoGripper auto = new autoGripper();
        auto.init(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        drive.setPoseEstimate(new Pose2d(42.0, 72.0, Math.toRadians(270.0)));
        //  drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, true));

        waitForStart();
        auto.initplate(hardwareMap);
        sleep(1000);

        if (isStopRequested()) return;

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .setReversed(false)
                            .splineTo(new Pose2d(46,28, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(270),Math.toRadians(95)))
                            .build()
            );

        auto.ClawDown(hardwareMap);
        sleep(1000);

            drive.turnSync(Math.toRadians(90));

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .strafeRight(55.0)
                    .build()
            );

            drive.turnSync(Math.toRadians(7));

            auto.ClawUp(hardwareMap);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(20.0)
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(40)
                        .build()
        );

    }
}
