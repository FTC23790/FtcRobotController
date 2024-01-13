package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "compTeleOp", group="Linear OpMode")

public class compTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftEH0 = null;
    private DcMotor frontRightCH0 = null;
    private DcMotor backRightCH1 = null;
    private DcMotor backLeftEH1 = null;

    private DcMotor linearSlideMotor = null;
    private DcMotor armJointMotor = null;

    private CRServo clawServo = null;
    private Servo wristJointServo = null;

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // rev core hex motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;

    double drive_speed_multiplier = 0.75;
    double lift_speed_multiplier = 1; //linear slide speed
    double arm_joint_speed_multiplier = 0.65;
    double claw_speed_multiplier = 0.5;



    @Override
    public void runOpMode() {

        frontLeftEH0 = hardwareMap.get(DcMotor.class, "frontleft0");
        frontRightCH0 = hardwareMap.get(DcMotor.class, "frontright0");
        backRightCH1 = hardwareMap.get(DcMotor.class, "backright1");
        backLeftEH1 = hardwareMap.get(DcMotor.class, "backleft1");

        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearslide");
        armJointMotor = hardwareMap.get(DcMotor.class, "armJointMotor");
        clawServo = hardwareMap.get(CRServo.class, "clawJointMotor");
        wristJointServo = hardwareMap.get(Servo.class, "wristJointMotor");


        //set direction for the motors
        frontLeftEH0.setDirection(DcMotor.Direction.FORWARD);
        frontRightCH0.setDirection(DcMotor.Direction.REVERSE);
        backRightCH1.setDirection(DcMotor.Direction.FORWARD);
        backLeftEH1.setDirection(DcMotor.Direction.FORWARD);

        linearSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        armJointMotor.setDirection(DcMotor.Direction.FORWARD);

        armJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armJointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        DcMotorControllerEx armJointMotorEx = (DcMotorControllerEx)armJointMotor.getController();
        PIDCoefficients pidOrig = armJointMotorEx.getPIDCoefficients(3, DcMotor.RunMode.RUN_USING_ENCODER);

        PIDCoefficients pidNew = new PIDCoefficients(2.5, 0.1, 0.2);
        armJointMotorEx.setPIDCoefficients(3, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        armJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {

            telemetry.addData("PID original kP:", pidOrig.p);
            telemetry.addData("PID original: kI", pidOrig.i);
            telemetry.addData("PID original: kD", pidOrig.d);
            double max;

            double x = gamepad1.left_stick_x;  //x component of the motor rotation, mapped to horizontal movemnet of left joystick
            double y = -1 * gamepad1.left_stick_y;  //y component, mapped to the vertical movement of the left joystick
            double rotation = gamepad1.right_stick_x; //rotational component, mapped to horizontal movement of right joystick


            double frontLeftPower = x + y + rotation;
            double backLeftPower = (-1 * x) + y + rotation;
            double frontRightPower = x - y + rotation;
            double backRightPower = (-1 * x) - y + rotation;


            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }


            frontLeftEH0.setPower(drive_speed_multiplier * frontLeftPower);
            frontRightCH0.setPower(drive_speed_multiplier * frontRightPower);
            backRightCH1.setPower(drive_speed_multiplier * backRightPower);
            backLeftEH1.setPower(drive_speed_multiplier * backLeftPower);


            // double lift_power = gamepad2.left_bumper ? 1.0 : 0.0; // linear slide speed: if left bumper pressed, speed = 1 else speed = 0
            // lift_power = gamepad2.right_bumper ? -1.0 : lift_power; // right bumper pressed, speed = -1
            // linearSlideMotor.setPower(lift_speed_multiplier * lift_power);

            if(gamepad2.dpad_left){
                linearSlideMotor.setTargetPosition(288);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(lift_speed_multiplier);
            }
            if(gamepad2.dpad_right){
                linearSlideMotor.setTargetPosition(0);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(lift_speed_multiplier);
            }

//            double arm_joint_power = gamepad2.dpad_up ? 1.0 : 0.0; // arm rotate speed: if up button, speed = 1 else speed = 0
//            arm_joint_power = gamepad2.dpad_down ? -1.0 : arm_joint_power; // right bumper pressed, speed = -1
//            armJointMotor.setPower(arm_joint_speed_multiplier * arm_joint_power);

            double claw_speed = gamepad2.a ? 1 : 0; // arm angle: if a button pressed, position = 0.25(270) else position = 0
            claw_speed = gamepad2.b ? -1 : claw_speed; // if b button pressed, position =  0 degrees
            clawServo.setPower(claw_speed_multiplier * claw_speed);

            if(gamepad2.dpad_up){
                armJointMotor.setTargetPosition(72);
                armJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armJointMotor.setPower(arm_joint_speed_multiplier);
            } else if(gamepad2.dpad_down){
                armJointMotor.setTargetPosition(0);
                armJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armJointMotor.setPower(arm_joint_speed_multiplier);
            }

            double wrist_joint_position = gamepad2.y ? 1.0 : 0.6; // wrist angle: if y button pressed, position = 0.25(270) else position = 0
            wrist_joint_position = gamepad2.x ? 0.0 : wrist_joint_position; // x button pressed, position =  0 degrees
            wristJointServo.setPosition(wrist_joint_position);



        }
    }

}
