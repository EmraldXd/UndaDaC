package org.firstinspires.ftc.teamcode.init;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.claw;
import org.firstinspires.ftc.teamcode.action.linearSlides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Colored Auto")
public class RoadRunnerAutoTwo extends OpMode {
    org.firstinspires.ftc.teamcode.action.linearSlides linearSlides = new linearSlides();
    ElapsedTime actionRuntime = new ElapsedTime();
    ElapsedTime autoRuntime = new ElapsedTime();
    TrajectorySequence blue;
    SampleMecanumDrive drive;
    boolean unrun;
    double offsetY = 12.98;
    double offsetX = 62.06;
    double offsetHead = 0;

    public void init() {
        linearSlides.init(this);

        Pose2d initialPose = new Pose2d(-12.98, 62.06, Math.toRadians(90.00));
        drive = new SampleMecanumDrive(hardwareMap, initialPose);
    }

    public void start() {
        blue = drive.trajectorySequenceBuilder(new Pose2d(-12.98, 62.06, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-34.04, 1.85), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(53.17, 24.99, Math.toRadians(-0.78)))
                .lineTo(new Vector2d(37.30, 36.41))
                .lineTo(new Vector2d(12.09, 36.70))
                .lineTo(new Vector2d(12.83, 48.87))
                .lineTo(new Vector2d(51.53, 49.16))
                .lineTo(new Vector2d(9.71, 48.57))
                .lineTo(new Vector2d(11.20, 56.28))
                .lineTo(new Vector2d(51.39, 59.39))
                .splineToSplineHeading(new Pose2d(60.14, 34.33, Math.toRadians(268.25)), Math.toRadians(268.25))
                .splineTo(new Vector2d(34.04, 1.11), Math.toRadians(90.00))
                .lineTo(new Vector2d(38.93, 39.23))
                .lineTo(new Vector2d(10.01, 2.81))
                .lineTo(new Vector2d(52.42, 62.95))
                .splineToSplineHeading(new Pose2d(59.00, 36.56, Math.toRadians(251.57)), Math.toRadians(-90.00))
                .splineTo(new Vector2d(33.59, 3.78), Math.toRadians(86.50))
                .lineTo(new Vector2d(58.21, 56.43))
                .build();


        unrun = true;
    }

    @Override
    public void loop() {
        if(unrun) {
            drive.followTrajectorySequence(blue);
        }
    }
}
