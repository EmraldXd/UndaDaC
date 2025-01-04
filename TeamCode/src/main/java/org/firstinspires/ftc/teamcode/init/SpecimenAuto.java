package org.firstinspires.ftc.teamcode.init;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Import our used RoadRunnerActions
import org.firstinspires.ftc.teamcode.customAction.linearSlideRR;
import org.firstinspires.ftc.teamcode.customAction.clawRR;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class SpecimenAuto extends LinearOpMode{
    Action start;
    Action align;
    Action pickupNew;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.94, -62.36, Math.toRadians(-90.00)));
        clawRR claw = new clawRR(hardwareMap);
        linearSlideRR linearSlides = new linearSlideRR(hardwareMap);
        Actions.runBlocking(claw.initializer());

        //This runs us to the rungs to hang our preload specimen
        start = drive.actionBuilder(new Pose2d(11.94, -62.36, Math.toRadians(-90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(0.00, -42.00), Math.toRadians(90.00))
                .build();

        //This used in order to align our robot to hang the specimens
        align = drive.actionBuilder(new Pose2d(0.00, -42.00, Math.toRadians(-90.00)))
                .lineToYConstantHeading(-38)
                .waitSeconds(1)
                .build();

        pickupNew = drive.actionBuilder(new Pose2d(0.00, -38.00, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(35.96, -35.81), Math.toRadians(90.00))
                .setReversed(true)
                .splineTo(new Vector2d(48.12, -60), Math.toRadians(-90))
                .build();


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                        start,
                        new SequentialAction(
                                linearSlides.angleSlidesUp(),
                                claw.angle(),
                                linearSlides.runToHighRung()
                        )
                    ),
                    align,
                    linearSlides.home(),
                    pickupNew
                )
            );
    }
}
