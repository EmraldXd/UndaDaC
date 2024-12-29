package org.firstinspires.ftc.teamcode.init;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.action.linearSlides;

@Config
@Autonomous
public class neutralAuto extends OpMode{
    org.firstinspires.ftc.teamcode.action.linearSlides linearSlides = new linearSlides();
    ElapsedTime actionRuntime = new ElapsedTime();
    ElapsedTime autoRuntime = new ElapsedTime();
    Action test;
    MecanumDrive drive;
    Action path;
    boolean unrun;
    boolean unbuilt;
    Pose2d initialPose = new Pose2d(-12.23, -65.33, Math.toRadians(90.00));

    public void init() {
        linearSlides.init(this);

        drive = new MecanumDrive(hardwareMap, initialPose);
        unrun = true;
    }

    public void loop() {
        test = drive.actionBuilder(initialPose)
                /**.splineTo(new Vector2d(14.76, -50), Math.toRadians(90))
                 .splineTo(new Vector2d(14.76, -40), Math.toRadians(90)) */
                .splineToLinearHeading(new Pose2d(0.22, -34.92, Math.toRadians(90.00)), Math.toRadians(90.00))
                //.splineToLinearHeading(new Pose2d(0.00, -48.00, Math.toRadians(0)), Math.toRadians(0))
                .splineTo(new Vector2d(0.37, -33.59), Math.toRadians(90.00))
                .splineTo(new Vector2d(-49.16, -39.67), Math.toRadians(90.00))
                .splineTo(new Vector2d(-51.83, -50.50), Math.toRadians(225.00))
                .splineTo(new Vector2d(-59.39, -32.85), Math.toRadians(90.00))
                .splineTo(new Vector2d(-55.84, -50.35), Math.toRadians(225.00))
                .splineTo(new Vector2d(-62.21, -14.46), Math.toRadians(180.00))
                .splineTo(new Vector2d(-62.81, -41.45), Math.toRadians(180))
                .splineTo(new Vector2d(-24.84, -11.35), Math.toRadians(0.00))
                .build();


        if(unrun) {
            Actions.runBlocking(test);
            unrun = !unrun;
        }
    }

}
