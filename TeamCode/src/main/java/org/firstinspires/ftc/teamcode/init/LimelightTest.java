package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.action.mecanumDrive;

@TeleOp(name = "limelightTest", group = "Main")
public class LimelightTest extends OpMode{
    org.firstinspires.ftc.teamcode.action.mecanumDrive mecanumDrive = new mecanumDrive();
    String current;
    Limelight3A limelight;
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        mecanumDrive.init(this);
    }

    @Override
    public void loop() {
        if(gamepad1.b) {
            limelight.pipelineSwitch(0);
            current = "Red";
        } else if (gamepad1.y) {
            limelight.pipelineSwitch(1);
            current = "Yellow";
        } else if (gamepad1.x) {
            limelight.pipelineSwitch(3);
            current = "Blue";
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            if(tx > 8) {
                mecanumDrive.setPower(0, 0, .5);
            } else if (tx < -8) {
                mecanumDrive.setPower(0, 0, -.5);
            } else {
                mecanumDrive.setPower(0, 0, 0);
            }

            if(ta < 2.5) {
                mecanumDrive.setPower(0, 0, 0);
            } else {
                mecanumDrive.setPower(0, 0, 0);
            }

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
