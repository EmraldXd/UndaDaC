package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.action.mecanumDrive;
import org.firstinspires.ftc.teamcode.customAction.intakeRR;
import org.firstinspires.ftc.teamcode.customAction.clawRR;


@Disabled
@Autonomous
public class ActionTester extends LinearOpMode {
    private static final ElapsedTime driveTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive mecanumDrive = new mecanumDrive();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        mecanumDrive.init(this);

        Action forward = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .splineTo(new Vector2d(10, 0), Math.toRadians(0))
                .build();

        Action forwardAgain = drive.actionBuilder(new Pose2d(10, -10, 0))
                .splineTo(new Vector2d(20, -10), 0)
                .build();

        waitForStart();

        Actions.runBlocking(forward);

        driveTime.reset();

        while(driveTime.time() < 1.00) {
            mecanumDrive.setPower(-.75, 0, 0);
        }

        Actions.runBlocking(forwardAgain);
    }
}
