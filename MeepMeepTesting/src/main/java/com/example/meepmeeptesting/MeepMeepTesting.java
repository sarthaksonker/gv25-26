package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.75)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(68, -22,Math.toRadians(180) ))
//
//                .splineToLinearHeading(
//                        new Pose2d(0, 0, Math.PI / -6),
//                        Math.PI / 3
//                )

//                // Move to (0,0) WITHOUT changing heading
//                .splineToConstantHeading(
//                        new Vector2d(0, 0),
//                        Math.PI / 3
//                )
//
//                // THEN set heading at the end
//                .turn(-Math.PI / 6)

                        .lineToX(0)
                .splineToConstantHeading(
                        new Vector2d(0, 0),
                        Math.PI / 6
                )
                .turn(Math.PI / 7)


                        .waitSeconds(2)

                        .lineToX(10)
                        .turnTo(-Math.PI/2)

                        .lineToY(-46.5)
                .waitSeconds(3)

                .splineToConstantHeading(
                        new Vector2d(0, 0),
                        Math.PI / 6
                )
                .turn(-Math.PI / 4)



                        .waitSeconds(3)

                .splineToConstantHeading(
                        new Vector2d(0, 0),
                        Math.PI / 2
                )

                        .turnTo(0)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

 //.lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))

//                .splineTo(new Vector2d(8, 4), Math.PI / 6)   // small forward-right curve, and math.PI = a 180 degree turn so how ever much u divide by is how u will turn
//                .splineTo(new Vector2d(16, 10), Math.PI / 5)  // continue forward, slight turn
//                .splineTo(new Vector2d(30, 18), Math.PI / 2) // continue forward, gr

//                .splineToConstantHeading(new Vector2d(0, 0), Math.PI / -6)
//                .turn(Math.PI / 2) // example adjustment