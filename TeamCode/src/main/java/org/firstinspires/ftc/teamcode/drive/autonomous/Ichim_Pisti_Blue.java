package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.structure.Gripper;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Outtake;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Shieeet sooon", group = "Blue_Quarry_Incomplete")
public class Ichim_Pisti_Blue extends LinearOpMode {
    autoGripper auto = new autoGripper();

    Outtake outtake = new Outtake();
    Intake intake = new Intake();


    public void autoInit(){
        outtake.init(hardwareMap);
        intake.init(hardwareMap);
    }

    public void autoUpdate(){
        outtake.update(0);
        intake.update();
    }

   @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        opencvSkystoneDetector detector = new opencvSkystoneDetector();
       drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive, true));
       autoInit();

       drive.setPoseEstimate(new Pose2d(-30.0, -72.0, Math.toRadians(90.0)));

       while(isStarted()==false && isStopRequested() == false) {
           intake.switchToIN();
           outtake.switchToINIT();
           autoUpdate();
       }

        waitForStart();


        if (isStopRequested()) return;


       // LinearInterpolator interp = new LinearInterpolator(Math.toRadians(90), Math.toRadians(50));
        //CUB 3 - Colectare cub
       autoUpdate();
        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .setReversed(false)
                        .splineTo(new Pose2d(-20, -33, Math.toRadians(90)), new SplineInterpolator(Math.toRadians(90.00), Math.toRadians(120)))
                        .build()
        );
       intake.switchToSTOP();
       outtake.switchToGETSTONE();
       autoUpdate();


        drive.followTrajectorySync(

                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(25, -49, Math.toRadians(180)))
                        .build()
        );

       drive.followTrajectorySync(

               drive.trajectoryBuilder()
                       .setReversed(true)
                       .splineTo(new Pose2d(55, -35, Math.toRadians(270)), new SplineInterpolator(Math.toRadians(270), Math.toRadians(180)))
                       .build()
       );



        /*auto.outTake(hardwareMap);*/


       /*     while(!outtake.isRESET()){
                outtake.switchToAUTO();
                outtake.update(0);
                telemetry.addData("Mosa", "ankur");

            }
            while(outtake.isRESET()){
                outtake.update(0);
            }*/


      drive.followTrajectorySync(
               drive.trajectoryBuilder()
                       .setReversed(false)
                       .splineTo(new Pose2d(30, -50, Math.toRadians(180)))
                       .build()
       );

     /*  drive.followTrajectorySync(

               drive.trajectoryBuilder()
                       .setReversed(false)
                       .splineTo(new Pose2d(-55, -35, Math.toRadians(180)), new SplineInterpolator(Math.toRadians(180.0), Math.toRadians(120.0)))
                       .build()
       )*/









   }
}
