package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static Vector2d vec(double x, double y) {
        return new Vector2d(x, y);
    }


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-70, 0, 0))
                        .splineTo(vec(-34, 0), Math.toRadians(270))
                        .strafeTo(vec(-34, -28))
                        .strafeTo(vec(-70, -28))
                        .strafeTo(vec(-70, -54))
                        .strafeTo(vec(-70, -28))
                        .strafeTo(vec(-70, -28))
                .strafeTo(vec(-70, -28))
                .strafeTo(vec(-70, -54))
                .strafeTo(vec(-70, -28))
                .strafeTo(vec(-34, -28))
                                .build());
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}