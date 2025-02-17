package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Helper.Beak.newBeak;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.BucketAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ClawAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ViperAction;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name = "AutoBlueOB", group = "RoadRunner")
public class AutoBlueOB extends LinearOpMode {

    public static class Params {
        public boolean easy = false;
        public double y = 38;
    }

    public static Params PARAMS = new Params();
    private MecanumDrive drive;
    private ClawAction Roar;
    private ViperAction Tiger;
    private newBeak Paw;
    private BucketAction Fur;
    private double x = 0;

    public void runOpMode(){

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Roar = new ClawAction(hardwareMap);
        Tiger = new ViperAction(hardwareMap);
        Paw = new newBeak(hardwareMap);
        Fur = new BucketAction(hardwareMap);

        Roar.CloseGrip();
        Paw.autonStartPos();

        waitForStart();

        telemetry.addData("okay", "so code needs to push 7");
        telemetry.update();

        if(PARAMS.easy){
            forward();
            toPark();
        }
        else{
            toLine();
            while (x < 3) {
                moveBack();
                if(x == 0) {
                    PARAMS.y = 38;
                    markOne();
                } else if (x == 1){
                    PARAMS.y = 44;
                    markOne();
                } else if (x == 2){
                    PARAMS.y = 46;
                    markOne();
                }
                humanPlayer();
                GoBack();
                toPark();
                GoBack();
                Reverse();
                backToLine();
                x++;
            }
            forward();
            toParkLast();
            updateTelemetry(drive.pose.position);
        }
    }

    public void toLine(){
        //beginning position: ends at the sub
        Action movePos = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-27)
                .build();
        Actions.runBlocking(new ParallelAction(movePos,Tiger.perfBeforeDropOff()));

        Action extraMove = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-29.5)
                .build();
        Actions.runBlocking(new SequentialAction(extraMove, Tiger.perfClawDropOnSub(), Roar.placeOnSub()));

    }

    public void moveBack () {
        //move back a little bit
        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .lineToX(-20)
                .build();
        Actions.runBlocking(new SequentialAction(moveBack, Tiger.clawHumanGrab()));
    }

    public void markOne(){
        //move to mark
        Action lineM1 = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-21.5, PARAMS.y), Math.toRadians(180))
                .build();
        Actions.runBlocking(new SequentialAction(lineM1, Paw.autonReachOB()));
    }

    public void humanPlayer(){
        //drop off in human player zone
        Action Player = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-4)
                .build();
        Actions.runBlocking(new SequentialAction(Player, Fur.autonHuman(), Paw.dropToHuman()));
    }

    public void GoBack(){
        //give time for human player to pick up sample
        //move out of the zone
        Action back = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .lineToX(-14)
                .build();
        Actions.runBlocking(back);
    }
    public void Reverse(){
        //get ready to go to sub
        Action turnAgain = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .turnTo(260)
                .build();
        Actions.runBlocking(turnAgain);
    }

    public void backToLine(){
        //drop sample on sub
        Action backAgain = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(-29, -5), Math.toRadians(180))
                .build();
        Actions.runBlocking(new SequentialAction(backAgain, Tiger.perfClawDropOnSub(), Roar.placeOnSub()));

        Action wait = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .waitSeconds(100)
                .build();
        Actions.runBlocking((wait));
    }

    private void toPark(){
        //to pick sample from human zone
        Action moveBasket= drive.actionBuilder(drive.pose)
                .setReversed(true)
                // .splineTo(new Vector2d(-12, -48), Math.toRadians(-20))
                .strafeTo(new Vector2d(-1,48))
                .build();
        Actions.runBlocking(new SequentialAction(moveBasket, Roar.grabFromHuman(), Tiger.perfBeforeDropOff(), Fur.autonBucketDown()));

    }

    private void toParkLast(){
        //final park position
        Action moveBasket= drive.actionBuilder(drive.pose)
                .setReversed(true)
                // .splineTo(new Vector2d(-12, -48), Math.toRadians(-20))
                .strafeTo(new Vector2d(-1.5,48))
                .build();
        Actions.runBlocking(new SequentialAction(moveBasket, Tiger.clawHumanGrab()));
    }

    private void forward(){
        //move all the way back for final
        Action moveOut = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-4)
                .build();
        Actions.runBlocking(moveOut);
    }

    private void updateTelemetry(Vector2d pos) {
        telemetry.addLine("RoadRunner Auto Drive BLUE");
        telemetry.addData("Current x Position", pos.x );
        telemetry.addData("Current y Postion", pos.y);
        telemetry.update();

    }



}
