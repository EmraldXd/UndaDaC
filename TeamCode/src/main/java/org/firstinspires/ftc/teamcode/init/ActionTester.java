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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.customAction.linearSlideRR;


@Autonomous
public class ActionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        linearSlideRR linearSlide = new linearSlideRR(hardwareMap);

        waitForStart();

        Actions.runBlocking(
            new SequentialAction(
                linearSlide.angleSlidesUp(),
                linearSlide.angleSlidesDown()
            )
        );
    }
}