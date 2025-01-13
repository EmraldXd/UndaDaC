package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.util.ElapsedTime;

//Import our used RoadRunnerActions
import org.firstinspires.ftc.teamcode.customAction.linearSlideRR;
import org.firstinspires.ftc.teamcode.customAction.clawRR;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.io.SequenceInputStream;
import java.util.concurrent.TimeUnit;

@Autonomous
public class NeutralRR extends LinearOpMode{

    //These are the actions the robot takes individually
    Action start;
    Action align;
    Action pickupFirst;
    Action pickupSecond;
    Action pickupThird;
    Action firstRunToBasket;
    Action secondRunToBasket;
    Action lastRunToBasket;
    MecanumDrive finalDrive;

    Pose2d placeholder;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive startDrive = new MecanumDrive(hardwareMap, new Pose2d(11.94, 62.36, Math.toRadians(90.00)));
        clawRR claw = new clawRR(hardwareMap);
        linearSlideRR linearSlides = new linearSlideRR(hardwareMap);
        Actions.runBlocking(claw.initializer());

        //This runs us to the rungs to hang our preload specimen
        start = startDrive.actionBuilder(new Pose2d(11.94, 62.36, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0.00, 42.00), Math.toRadians(-90.00))
                .build();

        //This used in order to align our robot to hang the specimens
        align = startDrive.actionBuilder(new Pose2d(0.00, 42.00, Math.toRadians(90.00)))
                .lineToYConstantHeading(40)
                .waitSeconds(0.5)
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
                        linearSlides.home()
                )
        );


        //We now build the rest of the pathing now the robot knows were it is
        finalDrive = new MecanumDrive(hardwareMap, placeholder);

        pickupFirst = finalDrive.actionBuilder(new Pose2d(0.00, 40.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(47.5, 45), Math.toRadians(-90.00))
                .waitSeconds(0.5)
                .build();

        firstRunToBasket = finalDrive.actionBuilder((new Pose2d(47.5, 41, Math.toRadians(90))))
                .setReversed(true)
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();

        pickupSecond = finalDrive.actionBuilder(new Pose2d(48, 52, Math.toRadians(-135)))
                .splineTo(new Vector2d(59, 35), Math.toRadians(-90))
                .build();

        secondRunToBasket = finalDrive.actionBuilder(new Pose2d(-59, -35, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();

        pickupThird = finalDrive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(-135)))
                .splineTo(new Vector2d(55, 33), Math.toRadians(-30))
                .build();

        lastRunToBasket = finalDrive.actionBuilder(new Pose2d(-55, -33, Math.toRadians(30)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                pickupFirst,
                                linearSlides.angleSlidesDown(),
                                claw.angle()
                        ),
                        linearSlides.collectPos(),
                        new ParallelAction(
                                claw.angle(),
                                claw.open()
                        )
                )
        );

        sleep(500);

        Actions.runBlocking(claw.close());

        sleep(500);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                claw.angle(),
                                linearSlides.home()
                        ),
                        new ParallelAction(
                                firstRunToBasket,
                                linearSlides.angleSlidesUp()
                        ),
                        new ParallelAction(
                                linearSlides.runToHighBasket(),
                                claw.angle()
                        ),
                        claw.angle()
                )
        );

        sleep(500);

        Actions.runBlocking(claw.open());

        sleep(500);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                claw.angle(),
                                linearSlides.home()
                        ),
                        linearSlides.angleSlidesDown()
                )
        );
    }
}

