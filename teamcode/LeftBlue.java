package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "LeftBlue", group = "Linear Opmode")
public class LeftBlue extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private DigitalSensors digitalSensors = null;
    private PurpleTensorFlow purpleTensorFlow = null;
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
    private DigitalChannel slideSwitch1 = null;
    private DigitalChannel clawSwitch1 = null;
    private DigitalChannel clawSwitch2 = null;
    /**
     * This is the entry of our Op Mode.
     */
    @Override
    public void runOpMode() {
        //initalize hardware
        initHardware();
        waitForStart();
        /*
        //tensorflow find the cube
        String action;
        //sleep to find artifact
        sleep(4000);
        if (isArtifactDetected()){
            action = "r";
            moveBotStrafe(8,0,1,0);
        }
        else{
            moveBotStrafe(8,0,1,0);
            //sleep to find artifact
            sleep(4000);
            if (isArtifactDetected()){
                action = "c";
            } else {
                action = "l";
            }
        }
        telemetry.addData("artifact location", action);
        telemetry.update();
        //if no cube reverse 8 inches
        //tensorflow find the cube
        //if no cube here we know its on the third square
        //forward towards tower
        moveBotDrive(36,1,0,0);
        //turn to fully align with goal
        turnRight(270,10);
        if ("r".equalsIgnoreCase(action)){
            //set arm to position 1
        } else if ("c".equalsIgnoreCase(action)){
            //set arm to position 2
        } else {
            //set arm to position 3
        }
        moveBotDrive(8,1,0,0);
        //TODO open claw
        moveBotDrive(8,-1,0,0);
        //move back to where we started
        moveBotStrafe(36,0,-1,0);
        //turn to align with the opening
        turnLeft(90,5);
        //strafe left into the square
        moveBotStrafe(36,0,1,0);
        //go further into the loading dock
        moveBotDrive(24,1,0,0);
         */
        telemetry.clear();
        telemetry.addData("CS1", digitalSensors.isCS1AtLimit());
        telemetry.addData("CS2", digitalSensors.isCS2AtLimit());
        telemetry.addData("SS1", digitalSensors.isSS1AtLimit());
        telemetry.update();
    }

    private void initHardware() {
        //theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        //theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");

        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE);

        digitalSensors = new DigitalSensors();
        digitalSensors.init(hardwareMap);

        purpleTensorFlow = new PurpleTensorFlow();
        purpleTensorFlow.init(hardwareMap);
        // We are expecting the IMU to be attached to an I2C port (port 0) on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }
    private void moveBotStrafe(int inches, double stick1Y, double stick1X, double stick2X){
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
    private void moveBotDrive(int inches, double stick1Y, double stick1X, double stick2X){
        String telemetryholder = new String();
        double timeoutS = determineDriveTime(inches);
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

            trigmecanum.mecanumDrive(0,0, scaledSpeed, false, false);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesRemaining = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
        trigmecanum.mecanumDrive(0, 0, 0, false, false);
    }
    //TODO see comments in turnLeft
    //ZYX, XYZ
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

            trigmecanum.mecanumDrive(0,0, -scaledSpeed, false, false);
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
}
