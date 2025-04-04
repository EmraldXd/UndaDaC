package org.firstinspires.ftc.teamcode.init;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.mecanumDrive;
import org.firstinspires.ftc.teamcode.action.linearSlides;
import org.firstinspires.ftc.teamcode.action.claw;
import org.firstinspires.ftc.teamcode.action.roller;
@TeleOp (name = "!THIS IS TELEOP", group = "Main")
public class teleOp extends OpMode {
    mecanumDrive mecanumDrive = new mecanumDrive();
    linearSlides linearSlides = new linearSlides();
    claw claw = new claw();
    roller roller = new roller();
    boolean mode = true;
    ElapsedTime swapDelay = new ElapsedTime();
    private FtcDashboard dashboard;
    @Override
    public void init() {
        //Initialize our motors
        mecanumDrive.init(this);
        linearSlides.init(this);
        //roller.init(this);
        claw.upOrDown(linearSlides.moveUpOrDown());
    }

    public void start() {
        mecanumDrive.runWithoutEncoder();
        linearSlides.setSlides();
        dashboard = FtcDashboard.getInstance();
        claw.init(this);
    }

    @Override
    public void loop() {

        if(gamepad1.back || gamepad2.back) {
            if(swapDelay.time() > .75) {
                mode = !mode;
                swapDelay.reset();
            }
        }

        if(mode) {
            //Controls for mecanumDrive()
            mecanumDrive.slowMode(gamepad1.left_bumper);
            mecanumDrive.driversideDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper);
            mecanumDrive.telemetryOutput();
            //Controls for linearSlides()
            linearSlides.angMotorPower(gamepad2.dpad_up, gamepad2.dpad_down);
            linearSlides.slidePower(gamepad2.left_stick_y);
            //linearSlides.goToSpecimen(gamepad2.x, (gamepad2.dpad_down || gamepad2.dpad_up));
            linearSlides.telemetryOutput();
            linearSlides.slideCheck();
            linearSlides.resetSlides(gamepad2.y);
            //Controls for claw();
            claw.moveClaw(gamepad2.a, linearSlides.liftTime(), gamepad1.b);
            claw.useClaw(gamepad2.right_bumper);
            claw.rotateClaw(gamepad2.left_bumper);
            claw.showTelemetry();
            claw.lift(linearSlides.liftTime());
            claw.moveArm(linearSlides.moveUpOrDown(), gamepad2.back);
            claw.ejection(gamepad2.right_trigger);
            /*Controls for Intake
            roller.moveRoller(gamepad2.a);
            roller.intake(gamepad2.right_bumper, gamepad2.left_bumper);
            roller.lift(linearSlides.liftTime());
            roller.telemetry();  */

            TelemetryPacket packet = new TelemetryPacket();
            packet.field().setRotation(Math.toRadians(90));
            packet.put("pos", "test");
            dashboard.sendTelemetryPacket(packet);
        } else if (!mode) {
            //Controls for mecanumDrive()
            mecanumDrive.slowMode(gamepad2.left_bumper);
            mecanumDrive.driversideDrive(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_bumper);
            mecanumDrive.telemetryOutput();
            //Controls for linearSlides()
            linearSlides.angMotorPower(gamepad1.dpad_up, gamepad1.dpad_down);
            linearSlides.slidePower(gamepad1.left_stick_y);
            //linearSlides.goToSpecimen(gamepad2.x, (gamepad2.dpad_down || gamepad2.dpad_up));
            linearSlides.telemetryOutput();
            linearSlides.slideCheck();
            //Controls for claw();
            claw.moveClaw(gamepad1.a, linearSlides.liftTime(), gamepad2.b);
            claw.useClaw(gamepad1.right_bumper);
            claw.rotateClaw(gamepad1.left_bumper);
            linearSlides.resetSlides(gamepad1.y);
            claw.showTelemetry();
            claw.lift(linearSlides.liftTime());
            claw.moveArm(linearSlides.moveUpOrDown(), gamepad1.back);
            claw.ejection(gamepad1.right_trigger);
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

        telemetry.addData("CurrentMode: ", mode ? 0 : 1);
        telemetry.addData("Current Position: ", gamepad2.right_trigger);
    }
}
