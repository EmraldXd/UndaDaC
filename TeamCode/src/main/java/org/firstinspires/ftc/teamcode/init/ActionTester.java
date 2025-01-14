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
import org.firstinspires.ftc.teamcode.customAction.intakeRR;
import org.firstinspires.ftc.teamcode.customAction.clawRR;


@Autonomous
public class ActionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        linearSlideRR linearSlide = new linearSlideRR(hardwareMap);
        //intakeRR intake = new intakeRR(hardwareMap);
        clawRR claw = new clawRR(hardwareMap);

        Actions.runBlocking(claw.initializer());

        waitForStart();

        Actions.runBlocking(claw.open());

        sleep(5000);

        Actions.runBlocking(claw.close());
    }
}
