package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "SingleSplineAuto", group = "Auto")
public class SingleSplineAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Starting pose (same as MeepMeep / RR test)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        // Initialize drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        // Normal spline: heading stays at π/6 while moving
//                        .splineToLinearHeading(
//                                new Pose2d(0, 0, Math.PI / 6), // final pose including heading
//                                Math.PI / 6                       // approach heading along spline
//                        )
                        .lineToX(10)

                        // No turn needed — already facing π/6
                        .build()
        );

    }
}
