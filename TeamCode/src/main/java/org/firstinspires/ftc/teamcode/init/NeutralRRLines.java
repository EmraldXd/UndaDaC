package org.firstinspires.ftc.teamcode.init;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.action.mecanumDrive;
import org.firstinspires.ftc.teamcode.customAction.clawRR;
import org.firstinspires.ftc.teamcode.customAction.linearSlideRR;

@Autonomous
public class NeutralRRLines extends LinearOpMode{

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
                .strafeToConstantHeading(new Vector2d(11.94, 43.00))
                .strafeToConstantHeading(new Vector2d(0.00, 43.00))
                .build();

        //This used in order to align our robot to hang the specimens
        align = drive.actionBuilder(new Pose2d(0.00, 43.00, Math.toRadians(90.00)))
                .lineToYConstantHeading(41)
                .waitSeconds(0.5)
                .build();

        pickupFirst = drive.actionBuilder(new Pose2d(0.00, 40.00, Math.toRadians(90.00)))
                .strafeToLinearHeading(new Vector2d(43.5, 38), Math.toRadians(-90.00),
                    new TranslationalVelConstraint(30.0))
                .waitSeconds(0.5)
                .build();

        firstRunToBasket = drive.actionBuilder((new Pose2d(43.5, 42, Math.toRadians(-90))))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(42.5, 46.5), Math.toRadians(-135),
                        new TranslationalVelConstraint(30.0))
                .build();

        pickupSecond = drive.actionBuilder(new Pose2d(46, 51, Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(54, 45.5), Math.toRadians(-90),
                        new TranslationalVelConstraint(30.0))
                .build();

        secondRunToBasket = drive.actionBuilder(new Pose2d(54, 45.5, Math.toRadians(-90)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(46, 48), Math.toRadians(-135),
                        new TranslationalVelConstraint(30.0))
                .build();

        pickupThird = drive.actionBuilder(new Pose2d(-55, -55, Math.toRadians(-120)))
                .splineTo(new Vector2d(55, 33), Math.toRadians(-30))
                .build();

        lastRunToBasket = drive.actionBuilder(new Pose2d(-55, -33, Math.toRadians(30)))
                .setReversed(true)
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();

        clear = drive.actionBuilder(new Pose2d(48,  52, Math.toRadians(-135)))
                .splineTo(new Vector2d(46, 50), Math.toRadians(-135),
                        new TranslationalVelConstraint(30.0))
                .build();

        waitForStart();

        Actions.runBlocking(start);

        lastReadPosition = encoder.getCurrentPosition();
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
        mecanumDrive.setPower(0, 0, 0);


        Actions.runBlocking(
                new SequentialAction(
                        linearSlides.specimenAngle(),
                        claw.angle(),
                        linearSlides.runToHighRung()
                )
        );

        driveTime.reset();
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
        );
    }
}

