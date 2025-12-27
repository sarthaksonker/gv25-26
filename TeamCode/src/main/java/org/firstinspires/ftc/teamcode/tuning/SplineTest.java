package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        // Only MecanumDrive, no TankDrive confusion
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        // Spline sequence: move forward with gradual heading changes
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(8, 4), Math.PI / 6)   // small forward-right curve
                     .splineTo(new Vector2d(16, 8), Math.PI / 4)  // continue forward, slight turn
                     //   .splineTo(new Vector2d(24, 12), Math.PI / 3) // continue forward, gradual right
                        .build()
        );
    }
}
