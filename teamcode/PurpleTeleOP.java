package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Our TeleOp for controlling our motors, linear slide and claws.
 * We also can adjust the speed of our robot.
 */

@TeleOp(name="PurpleTeleOp", group="Linear Opmode")
public class PurpleTeleOP extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private long timeSinceLaunchActionWasPressed = System.currentTimeMillis();
    private long timeSinceIntakeActionWasPressed = System.currentTimeMillis();
    private long timeSinceKnockActionWasPressed = System.currentTimeMillis();

    // Declare our hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;
    private Servo theLaunchServo = null;
    private Servo theIntakeServo = null;
    private DcMotor theLaunchMotor = null;
    private DcMotor intake = null;
    private DistanceSensor sensorRange = null;
    private Rev2mDistanceSensor sensorTimeOfFlight = null;
    private NormalizedColorSensor colorSensor = null;
    private DistanceSensor topDistanceSensor = null;
    private DistanceSensor bottomDistanceSensor = null;


    private boolean lastResetState = false;
    private boolean curResetState  = false;
    private boolean isIntakeOn = false;
    private boolean isLaunchOn = false;

    // A timer helps provide feedback while calibration is taking place
    private ElapsedTime timer = new ElapsedTime();

    // Settings for our servo
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_HALFWAY_POSITION = (SERVO_MAX_POS - SERVO_MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            clawAction();
            driveAction();
            knockAction();
            intakeAction();
            launchAction();
            launchAction();
            distanceAction();
            //TODO options below
            //Open Claw#2 B

            telemetry.update();
        }
    }
     private void initHardware(){
         // Initialize the hardware variables. Note that the strings used here as parameters
         // to 'get' must correspond to the names assigned during the robot configuration
         // step (using the FTC Robot Controller app on the phone).
         leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
         rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
         theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
         theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");
         theLaunchServo = hardwareMap.get(Servo.class, "the_launch_servo");
         theIntakeServo = hardwareMap.get(Servo.class, "the_intake_servo");
         theLaunchMotor = hardwareMap.get(DcMotor.class, "the_launch_motor");

         intake = hardwareMap.get(DcMotor.class, "intake");

         // Most robots need the motor on one side to be reversed to drive forward
         // Reverse the motor that runs backwards when connected directly to the battery
         leftDrive.setDirection(DcMotor.Direction.REVERSE);
         rightDrive.setDirection(DcMotor.Direction.FORWARD);
         theClawMotor.setDirection(DcMotor.Direction.FORWARD);
         intake.setDirection(DcMotor.Direction.REVERSE);

         bottomDistanceSensor = hardwareMap.get(DistanceSensor.class, "bottom_distance");
         topDistanceSensor = hardwareMap.get(DistanceSensor.class, "top_distance");

     }

    //private int timesExecuted = 0;

    private void knockAction() {

        // hits ring into launcher then returns to original position
        if (gamepad2.y) {

            //timesExecuted++;
            theLaunchServo.setPosition(0.70);
            sleep(1000);
            theLaunchServo.setPosition(Servo.MIN_POSITION);
        }
    }

    private void launchAction() {
        long elapsedTime = System.currentTimeMillis() - timeSinceLaunchActionWasPressed;
        if (gamepad2.x && (elapsedTime > 250)) {
            isLaunchOn = !isLaunchOn;
            timeSinceLaunchActionWasPressed = System.currentTimeMillis();
        }
        if (isLaunchOn) {
            theLaunchMotor.setPower(.56);
        } else {
            theLaunchMotor.setPower(0);
        }
        sleep(100);
    }

    private void intakeAction() {
        long elapsedTime = System.currentTimeMillis() - timeSinceIntakeActionWasPressed;
        if (gamepad2.left_bumper && (elapsedTime > 250)) {
            isIntakeOn = !isIntakeOn;
            timeSinceIntakeActionWasPressed = System.currentTimeMillis();
        }
        if (isIntakeOn) {
            intake.setPower(-.51);
            theIntakeServo.setPosition(Servo.MAX_POSITION);
        } else {
            intake.setPower(0);
            theIntakeServo.setPosition(0.88);
        }
        //This is the driver override
        if (gamepad1.a) {
            theIntakeServo.setPosition(0.88);
        }
    }
    /**
     * Sets servo position depending on button input, A = Middle B = Grab X = Open
     */
    private void clawAction() {
        // close the claw
        //if (gamepad2.b) {
        //    theClawServo.setPosition(SERVO_MIN_POS);
        //}

        // open the claw
        if (gamepad2.a) {
            theClawServo.setPosition(SERVO_MAX_POS);
        }
        else {
            theClawServo.setPosition(SERVO_MIN_POS);
        }
        // Linear speed
        double power = -gamepad2.left_stick_y;
        // Slow down the robot by factor 5 or 2 when right bumper pressed
        if (!gamepad2.right_bumper) {
            power = power / 5;
        } else {
            power = power / 2;
        }
        theClawMotor.setPower(power);
    }
    /**
     * Calculates drive power for POV and outputs power to telemetry
     */
    private void driveAction() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0;
        double rightPower = 0;

        if (gamepad1.left_bumper) {
            // Set to tank mode
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        } else {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
        }
        // Slow down the robot by factor 5
        if (!gamepad1.right_bumper) {
            leftPower = leftPower / 2;
            rightPower = rightPower / 2;
        }
        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        // Telemetry wont be updated until update is called
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("deviceName", bottomDistanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", bottomDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("deviceName", topDistanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", topDistanceSensor.getDistance(DistanceUnit.CM)));
    }
}