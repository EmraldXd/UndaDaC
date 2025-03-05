package org.firstinspires.ftc.teamcode.init;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.DisplacementProfile;
import com.acmerobotics.roadrunner.DisplacementTrajectory;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
        Actions.runBlocking(claw.close());
        mecanumDrive.init(this);


        /*This runs us to the rungs to hang our preload specimen

        This code is no longer used

        start = drive.actionBuilder(new Pose2d(11.94, 62.36, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0.00, 43.00), Math.toRadians(-90.00))
                .build();
        */

        /*This used in order to align our robot to hang the specimens

        This code is no longer used

        align = drive.actionBuilder(new Pose2d(0.00, 43.00, Math.toRadians(90.00)))
                .lineToYConstantHeading(41)
                .waitSeconds(0.5)
                .build();

        */

        start = drive.actionBuilder(new Pose2d(11.94, 62.36, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(25.73, 45.6), Math.toRadians(-20))
                .splineTo(new Vector2d(50, 50), Math.toRadians(45),
                        new TranslationalVelConstraint(20))
                .build();

        pickupFirst = drive.actionBuilder(new Pose2d(50, 50.00, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(50, 40), Math.toRadians(-90.00),
                new TranslationalVelConstraint(30.0))
                .build();

        firstRunToBasket = drive.actionBuilder((new Pose2d(50, 40, Math.toRadians(-90))))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(50, 50), Math.toRadians(-135),
                        new TranslationalVelConstraint(30.0))
                .build();

        pickupSecond = drive.actionBuilder(new Pose2d(50, 50, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(55, 40), Math.toRadians(-90),
                        new TranslationalVelConstraint(30.0))
                .build();

        secondRunToBasket = drive.actionBuilder(new Pose2d(54, 45.5, Math.toRadians(-90)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(45, 48), Math.toRadians(-135),
                        new TranslationalVelConstraint(30.0))
                .build();

        pickupThird = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(-120)))
                .splineTo(new Vector2d(55, 33), Math.toRadians(-30))
                .build();

        lastRunToBasket = drive.actionBuilder(new Pose2d(-55, -33, Math.toRadians(30)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 50), Math.toRadians(45))
                .build();

        clear = drive.actionBuilder(new Pose2d(48,  52, Math.toRadians(-135)))
                .splineTo(new Vector2d(46, 50), Math.toRadians(-135),
                        new TranslationalVelConstraint(30.0))
                .build();

        waitForStart();

        /* lastReadPosition = encoder.getCurrentPosition();
        driveTime.reset();
        while (opModeIsActive()) {
            mecanumDrive.setPower(0, .75, 0);
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
        while (opModeIsActive() && driveTime.time() < 1.125) {
            telemetry.addData("Dist.Traveled: ", Math.abs(Math.abs(lastReadPosition) - Math.abs(encoder.getCurrentPosition())));
            telemetry.update();
            mecanumDrive.setPower(0, -0.75, 0);
        }
        mecanumDrive.setPower(0, 0, 0); */


        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            start,
                        new SequentialAction(
                                linearSlides.angleSlidesUp(),
                                claw.angle(),
                                linearSlides.runToHighBasket(),
                                claw.angle()
                        )
                    )
                )
        );

        sleep(750);

        Actions.runBlocking(claw.open());

        sleep(750);

        Actions.runBlocking(claw.angle());

        sleep(500);

        Actions.runBlocking(
                new SequentialAction(
                        linearSlides.home(),
                        new ParallelAction(
                                new SequentialAction(
                                        claw.angle(),
                                        linearSlides.angleSlidesDown(),
                                        linearSlides.collectPos()
                                ),
                                pickupFirst
                        ),
                        claw.angle(),
                        claw.open()
                )
        );

        sleep(750);

        Actions.runBlocking(claw.close());

        sleep(750);

        Actions.runBlocking(
                new SequentialAction(
                        claw.angle(),
                        new ParallelAction(
                                new SequentialAction(
                                        linearSlides.home(),
                                        linearSlides.angleSlidesUp(),
                                        claw.angle()
                                ),
                                firstRunToBasket
                        ),
                        linearSlides.runToHighBasket(),
                        claw.angle()
                )
        );

        sleep(750);

        Actions.runBlocking(claw.open());

        sleep(750);

        Actions.runBlocking(claw.angle());

        sleep(500);

        Actions.runBlocking(
                new SequentialAction(
                        linearSlides.home(),
                        claw.angle(),
                        new ParallelAction(
                                new SequentialAction(
                                        linearSlides.angleSlidesDown(),
                                        linearSlides.collectPos()
                                ),
                                pickupSecond
                        ),
                        claw.angle()
                )
        );

        sleep(750);

        Actions.runBlocking(claw.close());

        sleep(750);

        Actions.runBlocking(claw.angle());

        sleep(500);

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                linearSlides.home(),
                                linearSlides.angleSlidesUp()
                        ),
                        secondRunToBasket
                )
        );

        Actions.runBlocking(claw.angle());

        sleep(750);

        Actions.runBlocking(claw.open());

        sleep(750);

        Actions.runBlocking(claw.angle());

        sleep(500);

        Actions.runBlocking(linearSlides.home());


        /*driveTime.reset();
        while(opModeIsActive() && driveTime.time() < 0.5) {
            mecanumDrive.setPower(0, 0.7, 0);
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

        sleep(400);

        Actions.runBlocking(claw.close());

        sleep(400);

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
                        new SequentialAction(
                                claw.angle(),
                                linearSlides.runToHighBasket()
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
        ); */
    }
}

