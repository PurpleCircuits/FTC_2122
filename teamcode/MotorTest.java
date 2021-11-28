package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "MotorTest", group = "Linear Opmode")

public class MotorTest extends LinearOpMode {

    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;

    public void initHardware(){
        HardwareMap hwMap = hardwareMap;
        motorFrontLeft = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hwMap.get(DcMotor.class, "motorBackLeft");
        //TODO send in motor direction
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //set power zero
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        motorFrontLeft.setPower(1);
        sleep(1000);
        motorFrontLeft.setPower(0);
        sleep(1000);

        motorFrontRight.setPower(1);
        sleep(1000);
        motorFrontRight.setPower(0);
        sleep(1000);

        motorBackLeft.setPower(1);
        sleep(1000);
        motorBackLeft.setPower(0);
        sleep(1000);

        motorBackRight.setPower(1);
        sleep(1000);
        motorBackRight.setPower(0);
        sleep(1000);
    }
}
