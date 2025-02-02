package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.action.mecanumDrive;

@Disabled
@TeleOp
public class DriverOrientedDrivingTest extends OpMode {

    mecanumDrive mecanumDrive = new mecanumDrive();

    @Override
    public void init() {
        mecanumDrive.init(this);
    }

    @Override
    public void loop() {
        mecanumDrive.driversideDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.back);
        mecanumDrive.telemetryOutput();
    }
}
