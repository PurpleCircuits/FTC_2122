package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Trigmecanum {

    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;

    public double motor0Scaled;
    public double motor1Scaled;
    public double motor2Scaled;
    public double motor3Scaled;
    public double slowdown = 1.0;

    private ElapsedTime runtime = new ElapsedTime() ;

    HardwareMap hwMap;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    //how many shaft rotations to wheel rotations
    //TODO what is the shaft to wheel ratio? ticks for wheel (383.6 for 40 motor)
    double shaftToWheelRatio = 1;
    double ticksPerShaftRotation = 0;
    double ticksPerWheelRotation = shaftToWheelRatio * ticksPerShaftRotation;
    double lrOfset;
    double fbOfset;
    //TODO measure robot
    //robot dimensions
    double robotDiameter = 25.46; //TODO this is if the robot is exactly 18*18 may need to change
    double wheelDiameter = 3.75; //both in inches
    double wheelCircumference = 3.14 * wheelDiameter;

    public void init(HardwareMap ahwMap)  {
        hwMap = ahwMap;

        motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hwMap.get(DcMotor.class, "motorBackLeft");
        //TODO send in motor direction
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set power zero?
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
    public void mecanumDrive(double Stick1Y, double Stick1X, double Stick2X, boolean A, boolean Y)  {

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO Stick1X + - values
        double motor0Raw = Stick1Y + Stick1X - (Stick2X / 2);//-Stick1Y - Stick1X - (Stick2X / 2);
        double motor1Raw = -Stick1Y + Stick1X - (Stick2X / 2);//Stick1Y - Stick1X - (Stick2X / 2);
        double motor2Raw = Stick1Y - Stick1X - (Stick2X / 2);//Stick1Y + Stick1X - (Stick2X / 2);
        double motor3Raw = -Stick1Y - Stick1X - (Stick2X / 2);//-Stick1Y + Stick1X - (Stick2X / 2);

        //Find max motor raw values
        double rawMax = Math.max(Math.abs(motor0Raw), Math.max(Math.abs(motor1Raw), Math.max(Math.abs(motor2Raw), Math.abs(motor3Raw))));
        //If any motor power value is outside -1 to 1, scale all values
        if (rawMax > 1) {
            motor0Scaled = motor0Raw / rawMax;
            motor1Scaled = motor1Raw / rawMax;
            motor2Scaled = motor2Raw / rawMax;
            motor3Scaled = motor3Raw / rawMax;
        } else {
            motor0Scaled = motor0Raw;
            motor1Scaled = motor1Raw;
            motor2Scaled = motor2Raw;
            motor3Scaled = motor3Raw;
        }
        if(A) {
            slowdown = 0.2;
        }else if (Y) {
            slowdown = 0.5;
        }
        motorFrontLeft.setPower(motor0Scaled * slowdown);
        motorFrontRight.setPower(motor1Scaled * slowdown);
        motorBackLeft.setPower(motor2Scaled * slowdown);
        motorBackRight.setPower(motor3Scaled * slowdown);
        double mticks = motorFrontLeft.getCurrentPosition();
    }
    public void fieldOrientedDrive(double Stick1Y, double Stick1X, double Stick2X, boolean A, boolean Y, double imuHeading, boolean rightDpad, boolean upDpad, boolean downDpad) {

        double headingRadians = 0;
        double headingPower = 0;
        //Changes Degrees to Radians
        imuHeading = imuHeading * (Math.PI / 180);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Convert joystick directions to radians and power
        if (Stick1X == 0 && Stick1Y == 0) {
            headingRadians = 0;
            headingPower = 0;
        } else {
            headingRadians = Math.atan2(Stick1X, Stick1Y);
            headingPower = Math.min(Math.sqrt((Stick1X * Stick1X) + (Stick1Y * Stick1Y)), 1);
        }

        //calculate individual motor power
        double motor0Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;
        double motor1Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;
        double motor2Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;
        double motor3Raw = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - Stick2X * 0.5;

        double rawMax = Math.max(Math.abs(motor0Raw), Math.max(Math.abs(motor1Raw), Math.max(Math.abs(motor2Raw), Math.abs(motor3Raw))));

        //If any motor power value is outside -1 to 1, scale all values
        if (rawMax > 1) {
            motor0Scaled = motor0Raw / rawMax;
            motor1Scaled = motor0Raw / rawMax;
            motor2Scaled = motor0Raw / rawMax;
            motor3Scaled = motor0Raw / rawMax;
        } else {
            motor0Scaled = motor0Raw;
            motor1Scaled = motor0Raw;
            motor2Scaled = motor0Raw;
            motor3Scaled = motor0Raw;
        }

        //If A pressed slow robot
        if (A) {
            slowdown = 0.5;
        } else if (Y) {
            slowdown = 1.0;
        }
        if (rightDpad) {
            lrOfset = 0.25;
        }
        else {
            lrOfset = 0;
        }
        if (upDpad) {
            fbOfset = 0.25;
        }
        else if (downDpad) {
            fbOfset = -0.25;
        }
        else {
            fbOfset = 0;
        }

        motorFrontLeft.setPower(motor0Scaled * slowdown - lrOfset - fbOfset);
        //motorFrontRight.setPower(motor1Scaled * slowdown - lrOfset + fbOfset);
        //motorBackRight.setPower(motor2Scaled * slowdown + lrOfset + fbOfset);
        //motorBackLeft.setPower(motor2Scaled * slowdown + lrOfset - fbOfset);
    }
}
