package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Far Auto")
public class Auto_Blue_Far extends LinearOpMode {

    enum State {
        MOVE_TO_SHOOT_1,
        SHOOT_1,
        GET_SET_2,
        MOVE_TO_SHOOT_2,
        SHOOT_2,
        RESET_HEADING,
        DONE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(68, -22, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();
        if (isStopRequested()) return;

        State state = State.MOVE_TO_SHOOT_1;


        Action moveToShoot1 =
                drive.actionBuilder(startPose)
                        .lineToX(0)
                        .splineToConstantHeading(new Vector2d(0, 0), Math.PI / 6)
                        .turn(-Math.PI / 7)
                        .build();


        Action getSet2 =
                drive.actionBuilder(startPose)
                     //   .waitSeconds(2)
                        .lineToX(10)
                        .turn(-Math.PI / 3)
                        .lineToY(-46.5)
                        .waitSeconds(3)
                        .build();

        Action moveToShoot2 =
                drive.actionBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(0, 0), Math.PI / 6)
                        .turn(Math.PI / 4)
                        .build();

        Action resetHeading =
                drive.actionBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(0, 0), Math.PI / 2)
                        .turnTo(0)
                        .build();

        while (opModeIsActive() && state != State.DONE) {
            switch (state) {

                case MOVE_TO_SHOOT_1:
                    if (moveToShoot1.run(null)) break;
                    state = State.SHOOT_1;
                    break;

                case SHOOT_1:
                    sleep(2000);
                    state = State.GET_SET_2;
                    break;

                case GET_SET_2:
                    if (getSet2.run(null)) break;
                    state = State.MOVE_TO_SHOOT_2;
                    break;

                case MOVE_TO_SHOOT_2:
                    if (moveToShoot2.run(null)) break;
                    state = State.SHOOT_2;
                    break;

                case SHOOT_2:
                    sleep(2000);
                    state = State.RESET_HEADING;
                    break;

                case RESET_HEADING:
                    if (resetHeading.run(null)) break;
                    state = State.DONE;
                    break;
            }
        }
    }
}
