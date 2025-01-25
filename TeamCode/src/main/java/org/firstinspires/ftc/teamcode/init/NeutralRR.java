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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//Import our used RoadRunner Actions
import org.firstinspires.ftc.teamcode.customAction.linearSlideRR;
import org.firstinspires.ftc.teamcode.customAction.clawRR;

//Import used non-RoadRunner Actions
import org.firstinspires.ftc.teamcode.action.mecanumDrive;

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
    Action clear;
    DcMotor encoder;

    double lastReadPosition;
    private static final ElapsedTime driveTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.94, 62.36, Math.toRadians(90.00)));
        clawRR claw = new clawRR(hardwareMap);
        linearSlideRR linearSlides = new linearSlideRR(hardwareMap);
        mecanumDrive mecanumDrive = new mecanumDrive();
        Actions.runBlocking(claw.initializer());
        mecanumDrive.init(this);
        encoder = hardwareMap.get(DcMotor.class, "ParEncoder");


        //This runs us to the rungs to hang our preload specimen
        start = drive.actionBuilder(new Pose2d(11.94, 62.36, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0.00, 43.00), Math.toRadians(-90.00))
                .build();

        //This used in order to align our robot to hang the specimens
        align = drive.actionBuilder(new Pose2d(0.00, 43.00, Math.toRadians(90.00)))
                .lineToYConstantHeading(41)
                .waitSeconds(0.5)
                .build();

        pickupFirst = drive.actionBuilder(new Pose2d(0.00, 40.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(47.5, 43), Math.toRadians(-90.00))
                .waitSeconds(0.5)
                .build();

        firstRunToBasket = drive.actionBuilder((new Pose2d(47.5, 43, Math.toRadians(-90))))
                .setReversed(true)
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();

        pickupSecond = drive.actionBuilder(new Pose2d(48, 52, Math.toRadians(-135)))
                .splineTo(new Vector2d(58, 43), Math.toRadians(-90))
                .build();

        secondRunToBasket = drive.actionBuilder(new Pose2d(56, 41, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();

        pickupThird = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(-135)))
                .splineTo(new Vector2d(55, 33), Math.toRadians(-30))
                .build();

        lastRunToBasket = drive.actionBuilder(new Pose2d(-55, -33, Math.toRadians(30)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();

        clear = drive.actionBuilder(new Pose2d(48,  52, Math.toRadians(-135)))
                .splineTo(new Vector2d(46, 50), Math.toRadians(-135))
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                start,
                                new SequentialAction(
                                        linearSlides.angleSlidesUp(),
                                        claw.angle()
                                )
                        )
                )
        );

        lastReadPosition = encoder.getCurrentPosition();
        driveTime.reset();
        while (opModeIsActive()) {
            mecanumDrive.setPower(0, 1, 0);
            telemetry.addData("delta pos: ", encoder.getCurrentPosition() - lastReadPosition);
            telemetry.update();
            if ((driveTime.time() >= 0.200) && (encoder.getCurrentPosition() - lastReadPosition >= -2)) {
                //telemetry.addData("delta pos: ", "break");
                //telemetry.update();
                break;
            }
            telemetry.addData("delta pos: ", encoder.getCurrentPosition() - lastReadPosition);
            telemetry.update();
            lastReadPosition = encoder.getCurrentPosition();
        }

        driveTime.reset();
        lastReadPosition = encoder.getCurrentPosition();
        while (opModeIsActive() && Math.abs(Math.abs(lastReadPosition) - Math.abs(encoder.getCurrentPosition())) < 2360) {
            telemetry.addData("Dist.Traveled: ", Math.abs(Math.abs(lastReadPosition) - Math.abs(encoder.getCurrentPosition())));
            telemetry.update();
            mecanumDrive.setPower(0, -1, 0);
        }
        mecanumDrive.setPower(0, 0, 0);

        driveTime.reset();
        while(opModeIsActive() && driveTime.time() < 0.25) {
            mecanumDrive.setPower(0, 1, 0);
        }
        mecanumDrive.setPower(0, 0, 0);

        Actions.runBlocking(
                new SequentialAction(
                        linearSlides.home(),
                        new ParallelAction(
                                pickupFirst,
                                new SequentialAction(
                                        new ParallelAction(
                                                linearSlides.angleSlidesDown(),
                                                claw.angle()
                                        ),
                                        linearSlides.collectPos(),
                                        new ParallelAction(
                                                claw.angle(),
                                                claw.open()
                                        )
                                )
                        )
                )
        );

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

        sleep(300);

        Actions.runBlocking(claw.open());

        sleep(500);

        Actions.runBlocking(claw.angle());

        sleep(200);

        Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction (
                                linearSlides.home(),
                                clear
                                ),

                        new ParallelAction(
                                linearSlides.angleSlidesDown(),
                                claw.angle(),
                                new SequentialAction(
                                         pickupSecond,
                                         linearSlides.collectPos(),
                                         claw.angle()
                                )
                        )
                )
        );

        sleep(500);

        Actions.runBlocking(claw.close());

        sleep(500);

        Actions.runBlocking(
                new SequentialAction(
                        claw.angle(),
                        linearSlides.home(),
                        new ParallelAction(
                                secondRunToBasket,
                                new SequentialAction(
                                        linearSlides.angleSlidesUp(),
                                        claw.angle()
                                )
                        ),
                        linearSlides.runToHighBasket(),
                        claw.angle()
                )
        );

        sleep(500);

        Actions.runBlocking(claw.open());

        sleep(500);

        Actions.runBlocking(claw.angle());

        sleep(300);

        Actions.runBlocking(
                new SequentialAction(
                        linearSlides.home(),
                        claw.angle(),
                        linearSlides.angleSlidesDown()
                )
        );
    }
}

