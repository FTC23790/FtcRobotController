package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
This is a copy of omniDriveMentor from onbot java that was made in order to
put into github
made on 11/17/2023

team 23750
*/
@TeleOp (name = "newTeleop", group="Linear OpMode")

public class newTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft0 = null;
    private DcMotor frontRight1 = null;
    private DcMotor backRight2 = null;
    private DcMotor backLeft3 = null;

    private DcMotor linearSlideMotor = null;

    double drive_speed_multiplier = 0.75;
    double lift_speed_multiplier = 0.4; //linear slide speed

    @Override
    public void runOpMode() {

        frontLeft0  = hardwareMap.get(DcMotor.class, "frontleft0");
        frontRight1 = hardwareMap.get(DcMotor.class, "frontright1");
        backRight2  = hardwareMap.get(DcMotor.class, "backright2");
        backLeft3   = hardwareMap.get(DcMotor.class, "backleft3");

        linearSlideMotor  = hardwareMap.get(DcMotor.class, "linearslide");



        //set direction for the motors
        frontLeft0.  setDirection(DcMotor.Direction.FORWARD);
        frontRight1. setDirection(DcMotor.Direction.FORWARD);
        backRight2.  setDirection(DcMotor.Direction.FORWARD);
        backLeft3.   setDirection(DcMotor.Direction.FORWARD);

        linearSlideMotor.    setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double max;

            double x = gamepad1.left_stick_x;  //x component of the motor rotation, mapped to horizontal movemnet of left joystick
            double y = -1 * gamepad1.left_stick_y;  //y component, mapped to the vertical movement of the left joystick
            double rotation = gamepad1.right_stick_x; //rotational component, mapped to horizontal movement of right joystick


            double frontLeftPower   =  x + y + rotation;
            double backLeftPower    = (-1 * x) + y + rotation;
            double frontRightPower  =  x - y + rotation;
            double backRightPower   = (-1 * x) - y + rotation;


            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }


            //for testing direction of motors
            //comment out if not testing
            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad2
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad2
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad2
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad2
            */


            frontLeft0.setPower(drive_speed_multiplier*frontLeftPower);
            frontRight1.setPower(drive_speed_multiplier*frontRightPower);
            backRight2.setPower(drive_speed_multiplier*backRightPower);
            backLeft3.setPower(drive_speed_multiplier*backLeftPower);


            double lift_power = gamepad1.left_bumper ? 1.0 : 0.0; // linear slide speed: if left bumper pressed, speed = 1 else speed = 0
            lift_power = gamepad1.right_bumper ? -1.0 : lift_power; // right bumper pressed, speed = -1
            linearSlideMotor.setPower(lift_speed_multiplier * lift_power);
        }

    }
}