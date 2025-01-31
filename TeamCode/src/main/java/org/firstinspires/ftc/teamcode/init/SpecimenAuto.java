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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//Import our used RoadRunnerActions
import org.firstinspires.ftc.teamcode.customAction.linearSlideRR;
import org.firstinspires.ftc.teamcode.customAction.clawRR;

//Import non-roadrunner things
import org.firstinspires.ftc.teamcode.action.mecanumDrive;

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
    DcMotor encoder;
    double lastReadPosition;
    private static final ElapsedTime driveTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-11.94, 62.36, Math.toRadians(90.00)));
        clawRR claw = new clawRR(hardwareMap);
        linearSlideRR linearSlides = new linearSlideRR(hardwareMap);
        Actions.runBlocking(claw.initializer());
        mecanumDrive mecanumDrive = new mecanumDrive();
        encoder = hardwareMap.get(DcMotor.class, "ParEncoder");


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

        pickupNew = drive.actionBuilder(new Pose2d(-62.5, 40.00, Math.toRadians(90.00)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-62.5,  61), Math.toRadians(-90))
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
                .splineToConstantHeading(new Vector2d(-48, 15), Math.toRadians(90))
                .strafeTo(new Vector2d(-48, 55))
                .strafeTo(new Vector2d(-48,15))
                .strafeTo(new Vector2d(-60, 15))
                .strafeTo(new Vector2d(-60, 55))
                .strafeTo(new Vector2d(-60,15))
                .strafeTo(new Vector2d(-62.5, 15))
                .strafeTo(new Vector2d(-62.5,55))
                .strafeTo(new Vector2d(-62.5,40))
                .build();


        mecanumDrive.init(this);

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
            mecanumDrive.setPower(0, 0.8, 0);
        }
        mecanumDrive.setPower(0, 0, 0);


        Actions.runBlocking(
                new SequentialAction(
                        linearSlides.home(),
                        pushSpecimens
                )
        );

        sleep(1000);

        Actions.runBlocking(
                new ParallelAction(
                        pickupNew,
                        linearSlides.angleSlidesDown(),
                        claw.angle()
                        )
        );
                        /*pickupNew,
                        linearSlides.take(),
                        moveFromWall,
                        hangNext,
                        linearSlides.runToHighRung(),
                        alignNext,
                        linearSlides.home(),
                        pickupSecond
                )
            );*/
    }
}
