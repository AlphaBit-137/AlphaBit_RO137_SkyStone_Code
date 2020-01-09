package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Op_Mode_Test", group="Linear Opmode")

public class OpModeTest extends LinearOpMode {

    // Declaram obiectul robot cu clasa hardware si timpul de rulare

    private ElapsedTime runtime = new ElapsedTime();
    Chassis DriveTrain = new Chassis();

    public DcMotor  RightFront   = null;    //Config: 0
    public DcMotor  RightBack  = null;      //Config: 1
    public DcMotor  LeftBack = null;        //Config: 2
    public DcMotor  LeftFront = null;       //Config: 3


    /* local OpMode members. */

    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */

    /* Initialize standard Hardware interfaces */


    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;



    @Override
    public void runOpMode() {

        DriveTrain.init(hardwareMap);

        LeftBack = hardwareMap.get(DcMotor.class, "Left_Back");
        RightFront = hardwareMap.get(DcMotor.class, "Right_Front");
        LeftFront = hardwareMap.get(DcMotor.class, "Left_Front");
        RightBack = hardwareMap.get(DcMotor.class, "Right_Back");

        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        LeftBack.setPower(0);
        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        waitForStart();

        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod

        while (opModeIsActive()) {
            double Front, Turn, Drive1, Drive2, Drive3, Drive4;

            //Primirea datelor de la joystick-uri

            double StrafeLeft = gamepad1.left_trigger;
            double StrafeRight = gamepad1.right_trigger;

            Front = gamepad1.left_stick_y;
            Turn = gamepad1.right_stick_x;

            Drive1 = Range.clip(Front - Turn, -1.0, 1.0);
            Drive2 = Range.clip(Front - Turn, -1.0, 1.0);
            Drive3 = Range.clip(Front + Turn, -1.0, 1.0);
            Drive4 = Range.clip(Front + Turn, -1.0, 1.0);


            //Calcularea puterii redate motoarelor

            if(gamepad1.left_stick_y != 0f || gamepad1.right_stick_x != 0f){
                DriveTrain.update(Drive1, Drive2,Drive3,Drive4);
            }else if(gamepad1.right_trigger != 0f || gamepad1.left_trigger != 0f){

                if (gamepad1.left_trigger != 0f){
                    DriveTrain.update(StrafeLeft, -StrafeLeft, StrafeLeft, -StrafeLeft);
                }

                if(gamepad1.right_trigger != 0f){
                    DriveTrain.update(-StrafeRight,StrafeRight,-StrafeRight, StrafeRight);
                }

            }else{
                DriveTrain.update(NULL_POWER, NULL_POWER, NULL_POWER, NULL_POWER);
            }



            if(gamepad1.a){
                DriveTrain.switchToFast();
            }
            if(gamepad1.b){
                DriveTrain.switchToSlow();
            }


            if(DriveTrain.RobotChasis == Chassis.ChassisModes.SLOW){
                telemetry.addData("Chassis", "1");
            }
            if(DriveTrain.RobotChasis == Chassis.ChassisModes.FAST){
                telemetry.addData("Chassis", "2");
            }


            telemetry.update();
        }
    }

}