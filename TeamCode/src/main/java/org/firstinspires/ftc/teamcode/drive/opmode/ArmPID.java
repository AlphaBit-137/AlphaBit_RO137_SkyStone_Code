package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ArmPID", group="Linear Opmode")

public class ArmPID extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx Arm;

    //private static final PIDFCoefficients PIDF = new PIDFCoefficients(1.174, 0.1174, 0.0, 11.74);


    @Override
    public void runOpMode(){

        Arm = hardwareMap.get(DcMotorEx.class, "Extindere");

        Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //Arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);
        Arm.setVelocityPIDFCoefficients(1.174,0.1174,0,11.74);
        Arm.setPositionPIDFCoefficients(5);

        while (opModeIsActive()){

            telemetry.addData("Encoder:", Arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
