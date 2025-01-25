package org.firstinspires.ftc.teamcode.init;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    Action alignNext;
    Action pickupNew;
    Action hangNext;
    Action moveFromWall;
    Action pickupSecond;
    Action hangSecond;
    Action alignSecond;
    Action pushSpecimens;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-11.94, 62.36, Math.toRadians(90.00)));
        clawRR claw = new clawRR(hardwareMap);
        linearSlideRR linearSlides = new linearSlideRR(hardwareMap);
        Actions.runBlocking(claw.initializer());


        //This runs us to the rungs to hang our preload specimen
        start = drive.actionBuilder(new Pose2d(-11.94, 62.36, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(0.00, 42.00), Math.toRadians(-90.00))
                .build();

        //This used in order to align our robot to hang the specimens
        align = drive.actionBuilder(new Pose2d(0.00, 42.00, Math.toRadians(90.00)))
                .lineToYConstantHeading(40)
                .waitSeconds(0.5)
                .build();

        pickupNew = drive.actionBuilder(new Pose2d(0.00, 40.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-40, 42), Math.toRadians(-90))
                .lineToYConstantHeading(61)
                .build();

        hangNext = drive.actionBuilder(new Pose2d(-38, 50, -90))
                .setReversed(true)
                .splineTo(new Vector2d(2, 44), Math.toRadians(-90))
                .build();

        alignNext = drive.actionBuilder(new Pose2d(2.00, 44.00, Math.toRadians(90.00)))
                .lineToYConstantHeading(40.5)
                .waitSeconds(0.5)
                .build();

        moveFromWall = drive.actionBuilder(new Pose2d(-40, 60, Math.toRadians(-90)))
                .lineToYConstantHeading(50)
                .build();

        pickupSecond = drive.actionBuilder(new Pose2d(2.00, 40.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-40, 42), Math.toRadians(-90))
                .lineToYConstantHeading(63)
                .build();

        hangSecond = drive.actionBuilder(new Pose2d(-40, 63, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(4, 44), Math.toRadians(-90))
                .build();

        alignSecond = drive.actionBuilder(new Pose2d(2.00, 44.00, Math.toRadians(90.00)))
                .lineToYConstantHeading(40.5)
                .waitSeconds(0.5)
                .build();

        pushSpecimens = drive.actionBuilder(new Pose2d(0, 41, Math.toRadians(90)))
                .splineTo(new Vector2d(-36.5, 24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-48, 10), Math.toRadians(90))
                .lineToY(55)
                .lineToY(10)
                .strafeTo(new Vector2d(-60, 10))
                .lineToY(55)
                .lineToY(10)
                .strafeTo(new Vector2d(-70, 10))
                .lineToY(55)
                .lineToY(10)
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
                        pushSpecimens
                        /*pickupNew,
                        linearSlides.take(),
                        moveFromWall,
                        hangNext,
                        linearSlides.runToHighRung(),
                        alignNext,
                        linearSlides.home(),
                        pickupSecond */
                )
            );
    }
}
