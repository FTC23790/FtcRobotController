package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 12 inches
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 */

@Autonomous(name="AutoCopy", group="Robot")

public class AutoEncoderSampleCopy extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private DcMotor frontLeftEH0 = null;
    private DcMotor frontRightCH0 = null;
    private DcMotor backRightCH1 = null;
    private DcMotor backLeftEH1 = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // rev 41-1291
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 90/25.4 ; // For figuring circumference- 90 mm converted to inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // leftDrive  = hardwareMap.get(DcMotor.class, "frontleft0");
        // rightDrive = hardwareMap.get(DcMotor.class, "frontright0");
        frontLeftEH0  = hardwareMap.get(DcMotor.class, "frontleft0");
        frontRightCH0 = hardwareMap.get(DcMotor.class, "frontright0");
        backRightCH1  = hardwareMap.get(DcMotor.class, "backright1");
        backLeftEH1   = hardwareMap.get(DcMotor.class, "backleft1");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // leftDrive.setDirection(DcMotor.Direction.REVERSE);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftEH0.  setDirection(DcMotor.Direction.REVERSE);
        frontRightCH0. setDirection(DcMotor.Direction.REVERSE);
        backRightCH1.  setDirection(DcMotor.Direction.FORWARD);
        backLeftEH1.   setDirection(DcMotor.Direction.REVERSE);


        frontLeftEH0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftEH1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightCH0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightCH1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftEH0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftEH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightCH0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightCH1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftEH0.getCurrentPosition(),
                frontRightCH0.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  36,  36, 5.0);  // S1: Forward 24 (??) Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeftEH0.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRightCH0.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            frontLeftEH0.setTargetPosition(newLeftTarget);
            backLeftEH1.setTargetPosition(newLeftTarget);
            frontRightCH0.setTargetPosition(newRightTarget);
            backRightCH1.setTargetPosition(newRightTarget);

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
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftEH0.isBusy() && frontRightCH0.isBusy())) {

                //Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeftEH0.getCurrentPosition(), frontRightCH0.getCurrentPosition());
                telemetry.update();
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
