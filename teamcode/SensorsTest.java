package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Autonomous(name = "SensorsTest", group = "Linear Opmode")
public class SensorsTest extends LinearOpMode {

    // The speed for the drive motors to operate at during autonomous
    private static final double SPEED = 0.3;

    // Declare hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    BNO055IMU imu;
    private DistanceSensor sensorRange = null;
    private Rev2mDistanceSensor sensorTimeOfFlight = null;


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
            driveFor(3.75, true);
        }
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.update();
        if (10 > sensorRange.getDistance(DistanceUnit.CM)){
            turnLeft(90, 5);
        } else {
            turnRight(270, 5);
        } //TODO Get the second distance sensor working
        //TODO Get encoderDrive working
        
        // Go forward away from wall for 36 inches
       /* if (opModeIsActive()) {
            driveFor(1, true);
        }
        if (opModeIsActive()) {
            turnLeft(90, 5);
        }
        if (opModeIsActive()) {
            driveFor(1, true);
        }
        if (opModeIsActive()) {
            turnRight(180, 5);
        }
        // Go forward 2 feet (24 inches)
        if (opModeIsActive()) {
            driveFor(1, false);
        }*/
       sleep(5000);
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
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
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
        while(opModeIsActive()
                && runtime.seconds() < timeoutS
                && degreesLeft>1
            // && oldDegreesLeft-degreesLeft>=0 //TODO possibly used as a 'stall' state - stuck up against something and stop turning
        )
        { //check to see if we overshot target
            scaledSpeed=degreesLeft/(10+degreesLeft)*speed;
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
        while(opModeIsActive() &&
                runtime.seconds() < timeoutS &&
                degreesLeft>1
                )
        { //check to see if we overshot target
            scaledSpeed=degreesLeft/(10+degreesLeft)*speed;
            if(scaledSpeed>1){scaledSpeed=.1;}

            leftDrive.setPower(scaledSpeed);
            rightDrive.setPower(-1*scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            degreesLeft = ((int)(Math.signum(targetHeading-angles.firstAngle)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))
                    + (int)(Math.signum(angles.firstAngle-targetHeading)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        }
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
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure to not run with encoder
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sensorRange = hardwareMap.get(DistanceSensor.class, "distance");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        // Log that init hardware is finished
        telemetry.log().clear();
        telemetry.log().add("Init. hardware finished.");
        telemetry.clear();
        telemetry.update();
    }
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

        // Rev2mDistanceSensor specific methods.
        telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        telemetry.update();
    }
}