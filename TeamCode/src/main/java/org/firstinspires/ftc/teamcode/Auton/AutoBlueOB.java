package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Helper.Beak.newBeak;
import org.firstinspires.ftc.teamcode.Helper.LEDColorHelper;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.BucketAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ClawAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ViperAction;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "AutoBlueOB", group = "RoadRunner")
public class AutoBlueOB extends LinearOpMode {

    public static class Params {
        public boolean easy = false;
        public String version = "15.6";
        public double y = 38.4;
        public double lastMoveX = -15;
        public double lastMoveY = 31;
        public double LastHeading = 35;

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

        telemetry.addData(PARAMS.version, "Drive OB Version" );
        telemetry.update();

        waitForStart();

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
            BobColor.setLEDColor(LEDColorHelper.LEDColor.ORANGE);
            turningOnOne();
            turningToTwo();

            updateTelemetry(drive.pose);
            FirstGo();
            backAndForth();
            FirstGo();
            BobColor.setLEDColor(LEDColorHelper.LEDColor.ORANGE);
            backAndForth();
        }
    }

    public void toLine(){
        //beginning position: ends at the sub
        Action extraMove = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-29.5)
                .build();
        Actions.runBlocking(new SequentialAction((new ParallelAction(Viper.fast_perfBeforeDropOff(), extraMove)), Viper.perfClawDropOnSub(), Claw.placeOnSub()));

    }

    public void moveBack () {
        //move back a little bit
        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .lineToX(-26)
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, Viper.clawHumanGrab(), Bucket.autonHuman()));
    }

    public void goMarkOne(){
        Action lineM1 = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-26.5, 21), Math.toRadians(130))
                .build();
        Actions.runBlocking(lineM1);
    }

    public void forwardOnOne(){
        Action MoreOne = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-27.1, 22.0), Math.toRadians(130))
                .build();
        Actions.runBlocking(new ParallelAction(MoreOne, Beak.autonPickupOB()));
    }

    public void turningOnOne(){
        Action Simple = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(PARAMS.lastMoveX, PARAMS.lastMoveY), Math.toRadians(PARAMS.LastHeading))
                .build();
        Actions.runBlocking(new SequentialAction(Simple, Beak.autonDropSampleToHuman()));
    }

    public void turningToTwo() {
        // Drive Sample Two and Pickup
        Action Pickup = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .turnTo(Math.toRadians(136))
                .splineToConstantHeading(new Vector2d(-28, 33.8), Math.toRadians(136))
                .build();
        Actions.runBlocking(new SequentialAction(Pickup, Beak.autonReachOB()));

        // Drive to Wall and Dump
        Action PickupTurn = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(3.5, 28), 0)
                .build();
        Actions.runBlocking(new ParallelAction(PickupTurn, Beak.autonPickupToSlide()));
        Actions.runBlocking(new SequentialAction(Beak.autonDropToHuman(), Claw.grabFromHuman(), new ParallelAction(Viper.fast_perfBeforeDropOff(), Bucket.autonBucketDown())));
    }

    public void FirstGo(){
        Action move = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-29, -3, Math.toRadians(0)), Math.toRadians(-180))
                .build();
        Actions.runBlocking(new SequentialAction(move, Viper.perfClawDropOnSub(), Claw.placeOnSub()));
    }

    public void backAndForth(){
        Action move2 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-24)
                .splineToSplineHeading(new Pose2d(3.5, 28, Math.toRadians(-180)), Math.toRadians(0))
                .build();
        Actions.runBlocking(new SequentialAction(new ParallelAction(move2, Viper.clawHumanGrab()), Claw.grabFromHuman()));
    }

    private void forward(){
        //move all the way back for final
        Action moveOut = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-4)
                .build();
        Actions.runBlocking(moveOut);
    }

    private void updateTelemetry(Pose2d pos) {
        telemetry.clear();
        telemetry.addLine("RoadRunner Auto Drive BLUE");
        telemetry.addData("Current x Position", pos.position.x );
        telemetry.addData("Current y Postion", pos.position.y);
        telemetry.addData("Current Heading", Math.toDegrees(pos.heading.imag) );
        telemetry.update();

    }



}
