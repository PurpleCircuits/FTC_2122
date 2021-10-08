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

@Autonomous(name = "RightBlue", group = "Linear Opmode")
public class RightBlue extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.5;
    private static final double COUNTS_PER_MOTOR_REV = 1120 ;    // (40 GEARBOX) eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;
    private BNO055IMU imu = null;
    private DistanceSensor topDistanceSensor = null;
    private DistanceSensor bottomDistanceSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel digitalTouch = null;
    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        //initalize hardware
        initHardware();
        waitForStart();
        //moveBotTime(1, -1, 0,0);
        //turnLeft(90, 5);
        //TODO speed issue with driving
        //TODO IMPORTANT! Stick 1Y negative is up
        //Vuforia magic find the duck
        //strafe right 48 inches


        moveBotTime(determineStrafeTime(36), 0, 1, 0);
            //forward 12 inches
        //TODO like last year (number of rings detection) depending on where the duck is were going to need change this next move in an else if
        moveBotTime(determineDriveTime(12), -1, 0, 0);
            //put the piece on the thing
        moveBotTime(determineDriveTime(12), 1, 0, 0);
        //turn right 45 using IMU
        turnRight(315,5);
        //moveBot(.5, 0,0, -1);
        //TODO go left for 30ish inches
        moveBotTime(determineDriveTime(36), 1, 0, 0);
        //TODO spin the board
        //re align ourselves square
        turnLeft(45, 5);
        //moveBot(.5, 0, 0, 1);
        //move right 24 to park in the loading dock
        moveBotTime(determineStrafeTime(18), 0, 1, 0);
    }

    private void initHardware() {
        //theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        //theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "limit_sensor");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE);

        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //bottomDistanceSensor = hardwareMap.get(DistanceSensor.class, "bottom_distance");
        //topDistanceSensor = hardwareMap.get(DistanceSensor.class, "top_distance");

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }
    private void moveBot(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineStrafeTime(inches);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
        telemetry.addData("Drive", telemetryholder);
        telemetry.update();
    }
    private void moveBotTime(double timeoutS, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetryholder = trigmecanum.mecanumDrive(stick1Y, stick1X, stick2X, false, false);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
        telemetry.addData("Drive", telemetryholder);
        telemetry.update();
    }
    public void turnLeft(double turnAngle, double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>2)
        {
            //Change the 10 on the line below to a variable
            scaledSpeed = degreesRemaining / (10 + degreesRemaining) * speed;
            if(scaledSpeed>1 || scaledSpeed<.5){scaledSpeed=.5;}//We have a minimum and maximum scaled speed

            trigmecanum.mecanumDrive(0,0, -scaledSpeed, false, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    //TODO see comments in turnLeft
    public void turnRight(double turnAngle, double timeoutS) {
        if (!opModeIsActive()){
            return;
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=1;
        double scaledSpeed=speed;
        double targetHeading = angles.firstAngle+turnAngle;
        if(targetHeading < -180) {targetHeading += 360;}
        if(targetHeading > 180){targetHeading -= 360;}
        double degreesRemaining = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS && degreesRemaining>2)
        {
            scaledSpeed=degreesRemaining/(10+degreesRemaining)*speed;
            if(scaledSpeed>1 || scaledSpeed<.5){scaledSpeed=.5;}//We have a minimum and maximum scaled speed

            trigmecanum.mecanumDrive(0,0, scaledSpeed, false, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    private double determineStrafeTime(int inches){
        double m = 21;
        return inches / m;
    }
    private double determineDriveTime(int inches){
        double m = 30;
        return inches / m;
    }

    //TODO Last years methods
    private String determineAction() {
        if (10 > topDistanceSensor.getDistance(DistanceUnit.CM)){
            return "c";
        } else if (10 > bottomDistanceSensor.getDistance(DistanceUnit.CM)){
            return "b";
        } else {
            return "a";
        }
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
}
