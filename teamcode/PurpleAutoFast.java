package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "PurpleAutoFast", group = "Linear Opmode")
public class PurpleAutoFast extends LinearOpMode {
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.5;
    private static final double COUNTS_PER_MOTOR_REV = 1120 ;    // (40 GEARBOX) eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;
    private BNO055IMU imu = null;
    private DistanceSensor topDistanceSensor = null;
    private DistanceSensor bottomDistanceSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel digitalTouch = null;
    private Servo theLaunchServo = null;
    private DcMotor theLaunchMotor = null;
    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "PurpleAutoFast");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //forward 26 inches
        encoderDrive(SPEED, 26,26, 10);
        //determine how many rings
        sleep (250);
        String action = determineAction();
        telemetry.addData("Action", action);
        telemetry.update();
        sleep (500);
        //reverse 10 inches
        encoderDrive(SPEED, -10, -10, 10);
        //turn right 45 degrees
        turnRight(315, 10);
        //forward 13 inches
        encoderDrive(SPEED, 13, 13, 10);
        // turn left 43 degrees
        turnLeft(43, 5);
        //begin spooling up the launch motor
        theLaunchMotor.setPower(.46);
        //drive forward 36 inches to launch line
        encoderDrive(SPEED, 36, 36, 10);
        //launch three rings
        launchAction();
        //drive forward 4 inches
        encoderDrive(SPEED,4, 4, 10);
        //stop to regain bearings
         sleep(500);
        //execute one of three pre-planned routes
        if ("a".equalsIgnoreCase(action)){
            processA();
        } else if ("b".equalsIgnoreCase(action)){
            processB();
        } else {
            processC();
        }
        telemetry.update();
        sleep(10000);
    }

    private void processA() {
        turnLeft(81, 10);
        encoderDrive(SPEED,24,24,20);
        dropGoal();
    }

    private void processB(){
        turnLeft(40, 10);
        encoderDrive(SPEED,15,15,20);
        dropGoal();
        encoderDrive(SPEED, -15, -15, 20);
    }

    private void processC() {
        turnLeft(37, 10);
        encoderDrive(SPEED,51,51,20);
        dropGoal();
        encoderDrive(SPEED, -40, -40, 20);
    }
    private void launchAction() {
        for (int i = 0; i<3; i++){
            // push knocker
            knockAction();
            // sleep 1 second
            sleep(750);
        }
        theLaunchMotor.setPower(0);
    }


        private void initHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "limit_sensor");
        theLaunchServo = hardwareMap.get(Servo.class, "the_launch_servo");
        theLaunchMotor = hardwareMap.get(DcMotor.class, "the_launch_motor");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Our robot needs the motor on one side to be reversed to drive forward
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        bottomDistanceSensor = hardwareMap.get(DistanceSensor.class, "bottom_distance");
        topDistanceSensor = hardwareMap.get(DistanceSensor.class, "top_distance");

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private String determineAction() {
        if (10 > topDistanceSensor.getDistance(DistanceUnit.CM)){
            return "c";
        } else if (10 > bottomDistanceSensor.getDistance(DistanceUnit.CM)){
            return "b";
        } else {
            return "a";
        }
    }
    public void turnLeft(double turnAngle, double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>2)
        {
            //TODO maybe change the 100 to 75 to make the turn slightly faster.
            //TODO change this is TestSensorsTest also
            scaledSpeed=degreesRemaining/(50+degreesRemaining)*speed;
            if(scaledSpeed>1 || scaledSpeed<.3){scaledSpeed=.3;}//We have a minimum and maximum scaled speed

            leftDrive.setPower(scaledSpeed);
            rightDrive.setPower(-1*scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    //TODO see comments in turnLeft
    public void turnRight(double turnAngle, double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesLeft = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && degreesLeft>2)
        {
            scaledSpeed=degreesLeft/(50+degreesLeft)*speed;
            if(scaledSpeed>1 || scaledSpeed<.3){scaledSpeed=.3;}

            leftDrive.setPower(-1*scaledSpeed);
            rightDrive.setPower(scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesLeft = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void dropGoal() {
        if (!opModeIsActive()) {
            return;
        }
        if(isAtLimit()) {
            //Arm down until sensor
            theClawMotor.setPower(-.3);
            while(isAtLimit()){
            }
            theClawMotor.setPower(0);
        }
        theClawServo.setPosition(SERVO_MAX_POS);
        //Arm Up until sensor
        theClawMotor.setPower(.5);
        sleep(500);
        theClawMotor.setPower(0);
    }
    private boolean isAtLimit(){
        // send the info back to driver station using telemetry function.
        // if the digital channel returns true it's HIGH and the button is unpressed.
        return digitalTouch.getState();
    }
    private void knockAction() {
        // hits ring into launcher then returns to original position
            theLaunchServo.setPosition(0.3);
            sleep(1000);
            theLaunchServo.setPosition(Servo.MIN_POSITION);
        }
}
