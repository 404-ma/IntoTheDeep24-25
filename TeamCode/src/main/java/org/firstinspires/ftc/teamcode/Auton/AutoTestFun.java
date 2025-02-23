package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Helper.Beak.newBeak;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.BucketAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ViperAction;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Disabled
@Autonomous(name = "AutoTestFun", group = "Test")
public class AutoTestFun extends LinearOpMode {
    public static class Params {
        public boolean DropOnSlide = false;
    }


    public static Params PARAMS = new Params();

    private newBeak Beak;
    private BucketAction Bucket;
    private MecanumDrive drive;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Beak = new newBeak(hardwareMap);
        Bucket = new BucketAction(hardwareMap);

        waitForStart();
        telemetry.addLine("Starting Test Sequence");
        telemetry.update();

        Beak.autonStartPos();
        if (PARAMS.DropOnSlide) {
            Bucket.DumpSample();
        }

        while (opModeIsActive()) {
            telemetry.addLine("Test Sequence Running...");
            telemetry.update();

            toLine();

            /*Bucket.StartPosition();

            if (!PARAMS.DropOnSlide) {
                Beak.autonReachSamp();
                sleep(2000);
                Bucket.DumpSample();
                sleep(500);
            } else {
                Beak.autonReachOB();
                sleep(500);
            }

            sleep(2000);

             */


        }

        // Move sample down
//        Viper.perfBeforeDropOff();
//        sleep(500);
//        Bucket.autonBucketDown();
//        sleep(500);

        // Drop off at human player zone
//        Beak.autonDropToHuman();
//        sleep(500);
//        sleep(500);

        telemetry.addLine("Test Sequence Complete");
        telemetry.update();
    }

    public void toLine(){
        //beginning position: ends at the sub
        Action extraMove = drive.actionBuilder(drive.pose)
                .setReversed(true)
                //.splineToLinearHeading()
                .splineToSplineHeading(new Pose2d(-1, 38, Math.toRadians(-90)), Math.toRadians(-180))
                .build();
        Actions.runBlocking(new SequentialAction());

    }
}
