package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *  much of this is directly copied from AutoDriveByEncoder sample code
 *  desired path:
 *
 */

@Autonomous(name="CompAuto", group="Robot")

public class compAuto extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor frontLeftEH0 = null;
    private DcMotor frontRightCH0 = null;
    private DcMotor backRightCH1 = null;
    private DcMotor backLeftEH1 = null;

    private ElapsedTime runtime = new ElapsedTime();

    //constants
    static final double COUNTS_PER_MOTOR_REV = 28;      // counts per revolution of dc motor rev 41-1291
    static final double DRIVE_GEAR_REDUCTION = 20.0;
    static final double WHEEL_DIAMETER_INCHES = 90 / 25.4; // For figuring circumference- 90 mm converted to inches
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /   //counts per inch of movement
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.6;

    @Override
    public void runOpMode() {

        //initialize motors
        frontLeftEH0 = hardwareMap.get(DcMotor.class, "frontleft0");
        frontRightCH0 = hardwareMap.get(DcMotor.class, "frontright0");
        backRightCH1 = hardwareMap.get(DcMotor.class, "backright1");
        backLeftEH1 = hardwareMap.get(DcMotor.class, "backleft1");

        //set polarity of motors
        frontLeftEH0.setDirection(DcMotor.Direction.REVERSE);
        frontRightCH0.setDirection(DcMotor.Direction.REVERSE);
        backRightCH1.setDirection(DcMotor.Direction.FORWARD);
        backLeftEH1.setDirection(DcMotor.Direction.REVERSE);

        //reset all encoders to 0
        frontLeftEH0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftEH1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightCH0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightCH1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftEH0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftEH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightCH0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightCH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at", "%7d :%7d",
                frontLeftEH0.getCurrentPosition(),
                frontRightCH0.getCurrentPosition());
        telemetry.update();

        waitForStart();


        //instructions for the robot
        //left drive goes about 1/12 less than right
        encoderDrive(DRIVE_SPEED, 38, 36, 5.0);  // S1: Forward 36 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout

    }

    /*
     *  Method (not part of the opmode) to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftEH0.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightCH0.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftEH1.getCurrentPosition() +  (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightCH1.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            frontLeftEH0.setTargetPosition(newFrontLeftTarget);
            backLeftEH1.setTargetPosition(newBackLeftTarget);
            frontRightCH0.setTargetPosition(newFrontRightTarget);
            backRightCH1.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftEH0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftEH1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightCH0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightCH1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftEH0.setPower(Math.abs(speed));
            backLeftEH1.setPower(Math.abs(speed));
            frontRightCH0.setPower(Math.abs(-speed));
            backRightCH1.setPower(Math.abs(-speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    //(runtime.seconds() < timeoutS) &&
                    (frontLeftEH0.isBusy() || frontRightCH0.isBusy() || backLeftEH1.isBusy() || backRightCH1.isBusy())) {

                //Display it for the driver.
                /*telemetry.addData("Running to", " %7d :%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeftEH0.getCurrentPosition(), frontRightCH0.getCurrentPosition());
                telemetry.update();*/
            }

            // Stop all motion;
            frontLeftEH0.setPower(0);
            backLeftEH1.setPower(0);
            frontRightCH0.setPower(0);
            backRightCH1.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftEH0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftEH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightCH0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightCH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
}