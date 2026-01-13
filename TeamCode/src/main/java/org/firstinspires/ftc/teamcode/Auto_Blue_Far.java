package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Far")
public class Auto_Blue_Far extends LinearOpMode {

    MecanumDrive drive;


    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(60, -22, Math.PI); // facing LEFT
        drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        if (isStopRequested()) return;

        // 1️⃣ DRIVE LEFT 3 inches
        Action driveLeft = drive.actionBuilder(startPose)
                .lineToX(57)
                .build();

        Actions.runBlocking(driveLeft);

        // 2️⃣ TURN IN PLACE (no translation drift)
        double oldAxial = MecanumDrive.PARAMS.axialGain;
        double oldLateral = MecanumDrive.PARAMS.lateralGain;

        MecanumDrive.PARAMS.axialGain = 0;
        MecanumDrive.PARAMS.lateralGain = 0;

        Action turn = drive.actionBuilder(drive.localizer.getPose())
                .turnTo(Math.PI / 1.5)
                .build();

        Actions.runBlocking(turn);

        MecanumDrive.PARAMS.axialGain = oldAxial;
        MecanumDrive.PARAMS.lateralGain = oldLateral;
    }
}
