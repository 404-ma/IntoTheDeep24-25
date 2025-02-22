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

@Autonomous(name = "AutoBlueInit", group = "RoadRunner")
public class AutoBlueInit extends LinearOpMode {

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

        if (PARAMS.easy) {
            forward();
        } else {
            toLine();
        }

//        if(PARAMS.easy){
//            forward();
//        }
//        else{
//            toLine();
//            moveBack();
//            goMarkOne();
//            forwardOnOne();
//            turningOnOne();
//            BobColor.setLEDColor(LEDColorHelper.LEDColor.ORANGE);
//            turningToTwo();
//            /*markOne();
//            humanPlayer();
//            Grabbing2();
//            //Grabbing3();
//            HumanToOB();
//            GoBack();
//            Reverse();
//            backToLine();
//            moveBack();
//            backAndForth();
//
//             */
//            //forward();
//            //toParkLast();
//            updateTelemetry(drive.pose.position);
//        }
    }

    public void toLine(){
        //beginning position: ends at the sub
        Action extraMove = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-29.5)
                .build();
//        Actions.runBlocking(new SequentialAction( (new ParallelAction(Viper.perfBeforeDropOff(), extraMove)), Viper.perfClawDropOnSub(), Claw.placeOnSub()));

        Actions.runBlocking(
                new SequentialAction(
                    (
                        (new ParallelAction(Viper.perfBeforeDropOff(), extraMove))
                    )
                )
        );

//        Actions.runBlocking(new SequentialAction( (Viper.perfClawDropOnSub()), Claw.placeOnSub()));

    }

    private void forward(){
        //move all the way back for final
        Action moveOut = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-4)
                .build();
        Actions.runBlocking(moveOut);
    }

}
