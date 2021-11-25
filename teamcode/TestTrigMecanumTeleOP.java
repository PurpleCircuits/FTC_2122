package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TestTrigMecanumTeleOP", group="Linear Opmode")
@Disabled
public class TestTrigMecanumTeleOP extends LinearOpMode {
    private Trigmecanum trigmecanum = null;
    private Servo theClawServo = null;
    BNO055IMU imu;
    private DistanceSensor distanceSensor = null;
    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_HALFWAY_POSITION = (SERVO_MAX_POS - SERVO_MIN_POS) / 2; // Start at halfway position
    private DcMotor theClawMotor = null;
//TODO Add motor and servo to test robot to use for test claw
    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        //theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");
        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD);
        //theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        //theClawMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            //distanceAction();
            //clawAction();
            trigmecanum.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.a, gamepad1.y);
            //trigmecanum.fieldOrientedDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.a, gamepad1.y, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("XYZ", angles.firstAngle);
            telemetry.update();
        }
    }
    private void distanceAction(){
        // generic DistanceSensor methods.
        telemetry.addData("deviceName", distanceSensor.getDeviceName() );
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
    }
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
}
