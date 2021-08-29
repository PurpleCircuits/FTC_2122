package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TrigMecanumTeleOP", group="Linear Opmode")
public class TrigMecanumTeleOP extends LinearOpMode {
    private final Trigmecanum trigmecanum = null;

    @Override
    public void runOpMode() throws InterruptedException {
        trigmecanum = new Trigmecanum();
        trigmecanum.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            trigmecanum.mecanumDrive(-gamepad1.left_sticck_y, gamepad1.left_stick_x,gamepad1.right_stick_x, gamepad1,a, gamepad1.y);
        }
    }
}
