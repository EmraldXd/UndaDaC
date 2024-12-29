package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp
public class RRLimelightTest extends OpMode{
    MecanumDrive drive;
    String current;
    Limelight3A limelight;
    double tx;
    double ta;
    double ty;
    TrajectoryActionBuilder turnRight;
    TrajectoryActionBuilder turnLeft;
    TrajectoryActionBuilder sitStill;
    Action search;
    int currentHeading;
    double currentY;
    boolean right;

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
        right = false;

        currentHeading = 0;

        Pose2d initPose = new Pose2d(0, 0, 0);

        drive = new MecanumDrive(hardwareMap, initPose);

        turnRight = drive.actionBuilder(initPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(0, currentY + 3), currentHeading);
        turnLeft = drive.actionBuilder(initPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(0, currentY + 3), currentHeading);
    }

    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)
        }

        if(tx > 8) {
            search = turnRight.build();
            right = true;
        } else if (tx < -8) {
            search = turnLeft.build();
            right = false;
        }

        if(search != null){
            Actions.runBlocking(
                    addY(right),
                    search
            );
        }
    }

    public Action addY(boolean isRight) {
        if(isRight) {
            currentY = currentY + 3;
        } else {
            currentY = currentY - 3;
        }
    }
}
