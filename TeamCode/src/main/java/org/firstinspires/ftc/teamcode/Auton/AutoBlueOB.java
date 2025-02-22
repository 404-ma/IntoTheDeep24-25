package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Helper.Beak.newBeak;
import org.firstinspires.ftc.teamcode.Helper.LEDColorHelper;
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
    private ClawAction Claw;
    private ViperAction Viper;
    private newBeak Beak;
    private BucketAction Bucket;
    private LEDColorHelper BobColor;
    private double x = 0;

    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Claw = new ClawAction(hardwareMap);
        Viper = new ViperAction(hardwareMap);
        Beak = new newBeak(hardwareMap);
        Bucket = new BucketAction(hardwareMap);
        BobColor = new LEDColorHelper(hardwareMap);

        waitForStart();

        telemetry.addData("okay", "so code needs to push 9");
        telemetry.update();

        BobColor.setLEDColor(LEDColorHelper.LEDColor.GREEN);
        Claw.CloseGrip();
        Beak.autonStartPos();

        if(PARAMS.easy){
            forward();
        }
        else{
            toLine();
            moveBack();
            goMarkOne();
            forwardOnOne();
            /*markOne();
            humanPlayer();
            Grabbing2();
            //Grabbing3();
            HumanToOB();
            GoBack();
            Reverse();
            backToLine();
            moveBack();
            backAndForth();

             */
            //forward();
            //toParkLast();
            updateTelemetry(drive.pose.position);
        }
    }

    public void toLine(){
        //beginning position: ends at the sub
        Action extraMove = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-29.5)
                .build();
        Actions.runBlocking(new SequentialAction( (new ParallelAction(Viper.perfBeforeDropOff(), extraMove)), Viper.perfClawDropOnSub(), Claw.placeOnSub()));

    }

    public void moveBack () {
        //move back a little bit
        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .lineToX(-27)
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, Viper.clawHumanGrab(), Bucket.autonHuman()));
    }

    public void goMarkOne(){
        Action lineM1 = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-27, 22), Math.toRadians(130))
                .build();
        Actions.runBlocking(new ParallelAction(lineM1, Beak.autonReachOB()));
    }

    public void forwardOnOne(){
        Action MoreOne = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-28, 23.5), Math.toRadians(130))
                .build();
        Actions.runBlocking(new SequentialAction(MoreOne, Beak.autonPickupOB()));
    }

    public void turningOnOne(){
        Action Turning = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-25, 26Math.toRadians(111))
                .build();
        Actions.runBlocking(new ParallelAction(Turning, Beak.autonSliderExtend()));
    }

    public void markOne(){
        //move to mark
        Action lineM1 = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-21, 36.8), Math.toRadians(180))
                .build();
        Actions.runBlocking(new SequentialAction(lineM1, Beak.autonReachOB()));
    }

    public void humanPlayer(){
        //drop off in human player zone
        Action Player = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-8)
                .build();
        Actions.runBlocking(new ParallelAction(Player, Beak.autonDropToHuman()));
    }

    public void Grabbing2(){
        //Grab from 2 mark
        Action Grab = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .strafeTo(new Vector2d(-21,47))
                .build();
        Actions.runBlocking(new SequentialAction(Grab, Beak.autonReachOB()));

    }

    public void Grabbing3(){
        //Grab from 3 mark
        Action Grab = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .turnTo(140)
                .build();
        Actions.runBlocking(new SequentialAction(Grab, Beak.grabAndDrop()));

    }

    public void HumanToOB(){
        //pick up hooked sample from wall
        Action PlayerGrab = drive.actionBuilder(drive.pose)
                .setReversed(true)
                //.turnTo(0)
                .lineToX(-1.5)
                .build();
        Actions.runBlocking(new SequentialAction(new ParallelAction(PlayerGrab, Beak.autonDropToHuman()), Claw.grabFromHuman(), new ParallelAction(Viper.perfBeforeDropOff(), Bucket.autonBucketDown())));
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
        Actions.runBlocking(new SequentialAction(backAgain, Viper.perfClawDropOnSub(), Claw.placeOnSub()));
    }

    public void backAndForth(){
        Action clawToOB = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-1, 37), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(clawToOB, Claw.grabFromHuman(), Viper.perfBeforeDropOff()));
    }

    private void toParkLast(){
        //final park position
        Action moveBasket= drive.actionBuilder(drive.pose)
                .setReversed(true)
                // .splineTo(new Vector2d(-12, -48), Math.toRadians(-20))
                .strafeTo(new Vector2d(-1.5,48))
                .build();
        Actions.runBlocking(new SequentialAction(moveBasket, Viper.clawHumanGrab()));
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
