package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="testbattery", group="Linear Opmode")
public class testbattery extends LinearOpMode {
    private DcMotor theSpinMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            spinAction();
        }
    }

    private void initHardware() {
        theSpinMotor = hardwareMap.get(DcMotor.class, "the_spin_motor");
    }
    private void spinAction(){
        double power = 0;
        double leftPower = gamepad2.left_trigger;
        double rightPower = gamepad2.right_trigger;
        if (leftPower > .1){
            if (leftPower > 1){
                power = .5;
            } else{
                power = leftPower;
            }
        } else {
            if (rightPower > .5){
                power = 1;
            } else{
                power = -rightPower;
            }

        }
        theSpinMotor.setPower(power);
    }
    private void urmomAction(){
        double power = 0;
        double leftPower = gamepad2.left_trigger;
        double rightPower = gamepad2.right_trigger;
        if (leftPower > .1){
            if (leftPower > 1){
                power = .5;
            } else{
                power = leftPower;
            }
        } else {
            if (rightPower > .5){
                power = 1;
            } else{
                power = -rightPower;
            }

        }
        theSpinMotor.setPower(power);
    }
//TODO Awesome year, it all worked in the end.
}