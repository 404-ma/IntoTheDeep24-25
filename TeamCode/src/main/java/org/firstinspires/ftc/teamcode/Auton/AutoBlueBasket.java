package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.Beak.newBeak;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ViperAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.BucketAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ClawAction;


@Autonomous(name = "Auto Blue Basket", group = "RoadRunner")
public class AutoBlueBasket extends LinearOpMode {

    public static class Params {
        public double versionNumber = 16.8;

    }

    //PICK UP FROM RANDOM SPOTS
    //ENDING POSITION
    //shorter starting point --> 3.5 inches
    public static Params PARAMS = new Params();
    private MecanumDrive drive;
    private newBeak arm;
    private ViperAction vip;
    private BucketAction bucket;
    private ClawAction claw;
    @Override
    public void runOpMode(){

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        arm = new newBeak(hardwareMap);
        vip = new ViperAction(hardwareMap);
        bucket = new BucketAction(hardwareMap);
        claw = new ClawAction(hardwareMap);

        //Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        telemetry.addLine("RoadRunner Auto Drive Basket Blue");
        telemetry.addLine();
        telemetry.addLine().addData("Version",PARAMS.versionNumber);
        telemetry.addLine();
        telemetry.update();

        claw.CloseGrip();
        arm.autonStartPos();

        waitForStart();
        telemetry.clear();
            toSub();
            toNewPosOne();
            toBasket();
            toPosTwo();
            toBasket();
            toPosThree();
            toBasket();
    }

    private void toSub(){
        Action extraMove = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-29.5)
                .build();
        Actions.runBlocking(new ParallelAction(vip.perfBeforeDropOff(), extraMove));

        Actions.runBlocking(new SequentialAction(vip.perfClawDropOnSub(), claw.placeOnSub()));
        //positioned back
        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-26.9)
                .build();
        Actions.runBlocking(new ParallelAction(moveBack,vip.autonReset()));
    }

    private void toNewPosOne(){
        Action moveOne = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-21, -39.1), Math.toRadians(180))
                .build();
        Actions.runBlocking(new SequentialAction(new SequentialAction(moveOne,  bucket.autonPrepForCatch()), arm.autonReachSamp()));
    }
    private void toPosTwo(){
        //pos two
        Action moveTwo = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-21, -49.1), Math.toRadians(180))
                .build();
        Actions.runBlocking(new SequentialAction((new ParallelAction (vip.autonReset(), moveTwo)), arm.autonReachSamp()));
    }

    private void toPosThree(){
        //pos three
        Action moveThree = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-24.9, -47.1), Math.toRadians(225))
                .build();
        Actions.runBlocking(new SequentialAction((new ParallelAction (vip.autonReset(), moveThree)), arm.autonReachSamp()));
    }
//
    private void toPosThreeTest2(){
        //pos three
        Action moveThree = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-36, -40.1), Math.toRadians(235))
                .build();
        Actions.runBlocking(new SequentialAction((new ParallelAction (vip.autonReset(), moveThree)), arm.autonReachSamp()));
    }
    private void toPosThreeTesting(){
        //pos three
        Action moveThree = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-24.8, -50.1), Math.toRadians(230))
                .build();
        Actions.runBlocking(new SequentialAction((new ParallelAction (vip.autonReset(), moveThree))));


    }
    private void toBasket(){
        //basket
        Action moveBasket= drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(-6.7, -48.3), Math.toRadians(-50.0))
                .build();
        Actions.runBlocking(new SequentialAction(moveBasket, vip.dumpSampleHighBasket(), bucket.autonPrepForCatch()) );
    }

    private void toParking(){
        Action movePark = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(-50, -20), Math.toRadians(90))
                .build();
        Actions.runBlocking(movePark);

    }

    private void dumbBasket(){
        Action moveOut = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-5)
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