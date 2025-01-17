package org.firstinspires.ftc.teamcode.init;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.action.mecanumDrive;
import org.firstinspires.ftc.teamcode.action.linearSlides;
import org.firstinspires.ftc.teamcode.action.claw;
import org.firstinspires.ftc.teamcode.action.roller;
@TeleOp (name = "THIS IS TELEOP", group = "Main")
public class teleOp extends OpMode {
    mecanumDrive mecanumDrive = new mecanumDrive();
    linearSlides linearSlides = new linearSlides();
    claw claw = new claw();
    roller roller = new roller();
    private FtcDashboard dashboard;
    @Override
    public void init() {
        //Initialize our motors
        mecanumDrive.init(this);
        linearSlides.init(this);
        claw.init(this);
        //roller.init(this);
    }

    public void start() {
        mecanumDrive.runWithoutEncoder();
        linearSlides.setSlides();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        //Controls for mecanumDrive()
        mecanumDrive.slowMode(gamepad1.left_bumper);
        mecanumDrive.driversideDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.back);
        mecanumDrive.telemetryOutput();
        //Controls for linearSlides()
        linearSlides.angMotorPower(gamepad2.dpad_up, gamepad2.dpad_down);
        linearSlides.slidePower(gamepad2.left_stick_y);
        //linearSlides.goToSpecimen(gamepad2.x, (gamepad2.dpad_down || gamepad2.dpad_up));
        linearSlides.telemetryOutput();
        //Controls for claw();
        claw.moveClaw(gamepad2.a, linearSlides.liftTime());
        claw.useClaw(gamepad2.right_bumper);
        claw.rotateClaw(gamepad2.dpad_left, gamepad2.dpad_right);
        claw.showTelemetry();
        claw.lift(linearSlides.liftTime());
        claw.moveArm(linearSlides.moveUpOrDown(), gamepad2.back);
        /*Controls for Intake
        roller.moveRoller(gamepad2.a);
        roller.intake(gamepad2.right_bumper, gamepad2.left_bumper);
        roller.lift(linearSlides.liftTime());
        roller.telemetry();  */

        TelemetryPacket packet = new TelemetryPacket();
        packet.field().setRotation(Math.toRadians(90));
        packet.put("pos", "test");
        dashboard.sendTelemetryPacket(packet);
    }
}
