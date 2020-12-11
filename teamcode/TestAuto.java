package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Most of this code was copied from the FTC examples, but we tweaked it for our purposes.
 *
 * This makes the robot drive forward and turn left and go forward, parking under the bridge during
 * autonomous gaining us 5 points.
 */
@Autonomous(name = "TestAuto", group = "Linear Opmode")
public class TestAuto extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 450 ;    // (20 GEARBOX) eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.3;

    // Declare hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    BNO055IMU imu;
    private DistanceSensor topDistanceSensor = null;
    private Rev2mDistanceSensor sensorTimeOfFlight = null;
    private DistanceSensor bottomDistanceSensor = null;

    // Used for determining how long something has ran
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "SensorsTest");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()){
            //driveFor(3.8, true);
            encoderDrive(.3, 36,36, 10);
        }
        sleep (250);
        String action = determineAction();
        telemetry.addData("Action", action);
        telemetry.update();
        sleep (2000);
        if (opModeIsActive()){
            encoderDrive(.3, -12, -12, 10);
        }
        // reverse robot 12 inches and turn right 270
        if (opModeIsActive()){
            turnRight(270, 5);
        }
        //Forward 24 inches
        if (opModeIsActive()){
            encoderDrive(.3, 24, 24, 10);
        }
        //turn left 90
        if (opModeIsActive()){
            turnLeft(90, 10);
        }
        if ("a".equalsIgnoreCase(action)){
            processA();
        } else if ("b".equalsIgnoreCase(action)){
            processB();
        } else {
            processC();
        }
        //Fetch how many rings were there and do one of three operations
        //if A execute processA
        //else if execute processB
        //else execute processC
        distanceAction();
        telemetry.update();
        sleep(10000);
    }

    /**
     * A simple method used to make our robot reverse or go forward.
     *
     * @param time The amount of time in seconds to drive for
     * @param forward True to go forward, false to go background
     */
    private void driveFor(double time, boolean forward) {
        // Determine the direction and power to set
        if (forward) {
            leftDrive.setPower(SPEED);
            rightDrive.setPower(SPEED);
        } else {
            leftDrive.setPower(-SPEED);
            rightDrive.setPower(-SPEED);
        }
        // Reset the timeout time and start motion
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            distanceAction();
            telemetry.update();
        }
        // Stop all motion
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    //TODO the below method is almost exact duplicate of the turnRight, minus the degrees left and setPower calls. can this be broken up differently?
    public void turnLeft(double turnAngle, double timeoutS) {
        //sleep(500);//TODO why?
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        //double oldDegreesLeft=turnAngle;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        //double oldAngle=angles.firstAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeoutS && degreesLeft>1
            // && oldDegreesLeft-degreesLeft>=0 //TODO possibly used as a 'stall' state - stuck up against something and stop turning
        )
        { //check to see if we overshot target
            //we changed the current 100 from our 10 because this will slow the robot down more
            //we think that this will fix the left turn, we changed it to 10 earlier
            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
            if(scaledSpeed>1){scaledSpeed=.1;}

            leftDrive.setPower(-1*scaledSpeed); //extra power to back wheels
            rightDrive.setPower(scaledSpeed); //due to extra weight
            //robot.leftFront.setPower(scaledSpeed);
            //robot.rightFront.setPower(-1*scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //oldDegreesLeft=degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
            //TODO below is questionable code based on the speed of CPU on the robot controller
            //if(Math.abs(angles.firstAngle-oldAngle)<1){speed*=1.1;} //bump up speed to wheels in case robot stalls before reaching target
            //oldAngle=angles.firstAngle;
        }
        //sleep(250); //small pause at end of turn TODO Why?
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    //TODO see comments in turnLeft
    public void turnRight(double turnAngle, double timeoutS) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesLeft = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && degreesLeft>1)
        { //check to see if we overshot target
            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
            if(scaledSpeed>1){scaledSpeed=.1;}

            leftDrive.setPower(scaledSpeed);
            rightDrive.setPower(-1*scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesLeft = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * A simple method used to turn our robot.
     *
     * @param time The amount of time in seconds to execute a turn for
     * @param right True to turn right, false to turn left
     */
    private void turnFor(double time, boolean right) {
        // Determine the direction and power to set
        if (right) {
            leftDrive.setPower(SPEED);
            rightDrive.setPower(-SPEED);
        } else {
            leftDrive.setPower(-SPEED);
            rightDrive.setPower(SPEED);
        }
        // Reset the timeout time and start motion
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time));
        // Stop all motion
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * Simply initializes our hardware from the FTC config into variables.
     */
    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Our robot needs the motor on one side to be reversed to drive forward
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Ensure to not run with encoder //TODO add back in after done with encoders (if not using)
        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
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
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("Bottom deviceName", bottomDistanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", bottomDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Top deviceName", topDistanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", topDistanceSensor.getDistance(DistanceUnit.CM)));

    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
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
    private void processA() {
        if (opModeIsActive())
            encoderDrive(.5,48,48,20);
    }
    private void processB(){
        if (opModeIsActive()) {
            encoderDrive(.5, 72, 72, 20);
        }
        if (opModeIsActive()){
            turnLeft(90, 10);
        }
        if (opModeIsActive()) {
            encoderDrive(.5,24,24,10);
        }
    }
    private void processC() {
        if (opModeIsActive()) {
            encoderDrive(.5, 96, 96, 20);
        }
    }
}