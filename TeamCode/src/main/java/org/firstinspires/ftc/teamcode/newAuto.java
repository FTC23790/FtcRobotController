package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "newAuto", group="Linear OpMode")


public class newAuto extends LinearOpMode {

    //encoder constants
    final double constCountsPerRev = 0;
    final double constWheelDiameter_mm = 90; //this is in millimeters
    final double constWheelDiameter_in = 90/25.4;
    final double constCountsPerInch = 20 * (constCountsPerRev) / (constWheelDiameter_in * 3.14104);

    //pid constants
    final double PID_kp = 1;
    final double PID_ki = 0;
    final double PID_kd = 0;


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft0 = null; //group 1
    private DcMotor frontRight1 = null; //group 2
    private DcMotor backRight2 = null; //group 1
    private DcMotor backLeft3 = null; //group 2

    private DcMotor liftLeft = null;
    private DcMotor liftRight = null;

    double drive_speed_multiplier = 0.75;
    double lift_speed_multiplier = 0.4;

    @Override
    public void runOpMode() {
        frontLeft0  = hardwareMap.get(DcMotor .class, "frontleft0");
        frontRight1 = hardwareMap.get(DcMotor.class, "frontright1");
        backRight2  = hardwareMap.get(DcMotor.class, "backright2");
        backLeft3   = hardwareMap.get(DcMotor.class, "backleft3");

        liftLeft  = hardwareMap.get(DcMotor.class, "lift_left");
        liftRight = hardwareMap.get(DcMotor.class, "lift_right");


        frontLeft0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        frontLeft0.setTargetPosition((int)(20*constCountsPerInch));
        backRight2.setTargetPosition((int)(20*constCountsPerInch));

        frontLeft0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()) {
            telemetry.addData("left position:", frontLeft0.getCurrentPosition());
            telemetry.addData("right position:", backRight2.getCurrentPosition());
            telemetry.update();
        }



    }


}
