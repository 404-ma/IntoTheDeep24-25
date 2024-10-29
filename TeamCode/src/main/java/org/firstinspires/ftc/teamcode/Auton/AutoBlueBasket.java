package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name = "Auto Blue Basket", group = "RoadRunner")
public class AutoBlueBasket extends LinearOpMode {

    public static class Params {
        public int versionNumber = 0;

    }

    public static Params PARAMS = new Params();
    private MecanumDrive drive;


    @Override
    public void runOpMode(){

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        telemetry.addLine("RoadRunner Auto Drive Basket Blue");
        telemetry.addLine();
        telemetry.addLine().addData("Version",PARAMS.versionNumber);
        telemetry.addLine();
        telemetry.update();

        waitForStart();
        telemetry.clear();

    }

    private void toSub(){
        //beginning position: ends at the sub
        Action movePos = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-44.5)
                .build();
        Actions.runBlocking(movePos);

        //TODO: add code for claw

        //positioned back
        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-24.5)
                .build();
        Actions.runBlocking(moveBack);
    }

    private void toPosOne(){
        //pos one
        Action moveOne = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(-44.5)
                .build();
        Actions.runBlocking(moveOne);

        //basket
    }

    private void toPosTwo(){
        //pos two
        //basket
    }

    private void toPosThree(){
        //pos three
        //basket
    }

    private void toBasket(){

        //basket
    }


}