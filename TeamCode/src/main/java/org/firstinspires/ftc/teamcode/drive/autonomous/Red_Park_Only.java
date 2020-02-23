package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Autonomous(name="RED_PARKING", group = "BLUE")
public class Red_Park_Only extends LinearOpMode {

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
                        .forward(15)
                        .strafeLeft(5)
                        .build()
        );
    }
}
