package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    private Servo armJointServo = null;
    private Servo wristJointServo = null;

    double drive_speed_multiplier = 0.75;
    double lift_speed_multiplier = 0.75; //linear slide speed

    @Override
    public void runOpMode() {

        frontLeft0  = hardwareMap.get(DcMotor.class, "frontleft0");
        frontRight1 = hardwareMap.get(DcMotor.class, "frontright1");
        backRight2  = hardwareMap.get(DcMotor.class, "backright2");
        backLeft3   = hardwareMap.get(DcMotor.class, "backleft3");

        linearSlideMotor  = hardwareMap.get(DcMotor.class, "linearslide");
        armJointServo = hardwareMap.get(Servo.class, "armJointMotor");
        wristJointServo = hardwareMap.get(Servo.class, "wristJointMotor");



        //set direction for the motors
        frontLeft0.  setDirection(DcMotor.Direction.REVERSE);
        frontRight1. setDirection(DcMotor.Direction.FORWARD);
        backRight2.  setDirection(DcMotor.Direction.REVERSE);
        backLeft3.   setDirection(DcMotor.Direction.REVERSE);

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
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad1
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad1
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad1
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad1
            */


            frontLeft0.setPower(drive_speed_multiplier*frontLeftPower);
            frontRight1.setPower(drive_speed_multiplier*frontRightPower);
            backRight2.setPower(drive_speed_multiplier*backRightPower);
            backLeft3.setPower(drive_speed_multiplier*backLeftPower);


            double lift_power = gamepad2.left_bumper ? 1.0 : 0.0; // linear slide speed: if left bumper pressed, speed = 1 else speed = 0
            lift_power = gamepad2.right_bumper ? -1.0 : lift_power; // right bumper pressed, speed = -1
            //to do: make sure that right and left bumper cant be pressed at the same time
            linearSlideMotor.setPower(lift_speed_multiplier * lift_power);

            double arm_joint_position = gamepad2.dpad_up ? 0.25 : 0.0; // arm angle: if d-pad up button pressed, position = 0.25(180) else position = 0
            arm_joint_position = gamepad2.dpad_down ? 0 : arm_joint_position; // d-pad down button pressed, position =  0 degrees
            armJointServo.setPosition(arm_joint_position);

            double wrist_joint_position = gamepad2.a ? 0.25 : 0.0; // wrist angle: if a up button pressed, position = 0.25(180) else position = 0
            wrist_joint_position = gamepad2.x ? 0 : wrist_joint_position; // x down button pressed, position =  0 degrees
            armJointServo.setPosition(wrist_joint_position);


        }

    }
}