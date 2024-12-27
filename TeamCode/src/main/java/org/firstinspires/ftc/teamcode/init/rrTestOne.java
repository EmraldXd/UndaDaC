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
@Autonomous(name = "TestOne", group = "Autonomous")
public class rrTestOne extends OpMode {

    org.firstinspires.ftc.teamcode.action.linearSlides linearSlides = new linearSlides();
    ElapsedTime actionRuntime = new ElapsedTime();
    ElapsedTime autoRuntime = new ElapsedTime();
    Action test;
    MecanumDrive drive;
    Action path;
    boolean unrun;
    boolean unbuilt;
    Pose2d initialPose = new Pose2d(14.76, -63.40, Math.toRadians(90));

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
                .lineToYLinearHeading(-48, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(36, -48), Math.toRadians(0))
                .splineTo(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42, -13), Math.toRadians(90))
                .lineToYConstantHeading(-48)
                .splineToConstantHeading(new Vector2d(30, -54), Math.toRadians(90))
                /*.splineTo(new Vector2d(37.15, -15.65), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(47.83, -14.16), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(47.38, -52.28), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(47.23, -7.34), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(58.36, -12.23), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(56.87, -51.98), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(58.06, -3.63), Math.toRadians(-90.00))
                .splineToConstantHeading(new Vector2d(62.66, -62.21), Math.toRadians(270.00)) */
                .build();

        if(unrun) {
            Actions.runBlocking(test);
            unrun = !unrun;
        }
    }
}
