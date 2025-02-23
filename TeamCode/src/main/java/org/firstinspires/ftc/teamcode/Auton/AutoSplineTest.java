package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helper.Beak.newBeak;
import org.firstinspires.ftc.teamcode.Helper.LEDColorHelper;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "Auto Spline Test", group = "Test")
public class AutoSplineTest extends LinearOpMode {
    public static class Params {
        public double MoveOneX = 29;
        public double MoveOneY = 34;
        public double MoveOneHeadingA = 180;
        public double MoveOneHeadingB = 0;

        public double MoveTwoX = 1;
        public double MoveTwoY = 0;
        public double MoveTwoHeadingA = 0;
        public double MoveTwoHeadingB = 180;
    }

    public static Params PARAMS = new Params();

    private MecanumDrive drive;
    private LEDColorHelper BobColor;
    private newBeak beak;

    public void runOpMode(){
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        beak = new newBeak(hardwareMap);
        BobColor = new LEDColorHelper(hardwareMap);

        waitForStart();

        BobColor.setLEDColor(LEDColorHelper.LEDColor.GREEN);
        beak.autonStartPos();

        telemetry.addLine("Starting Test Sequence");
        telemetry.update();

        Action move = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(PARAMS.MoveOneX, PARAMS.MoveOneY,
                        Math.toRadians(PARAMS.MoveOneHeadingA)), Math.toRadians(PARAMS.MoveOneHeadingB))
                .build();
        Actions.runBlocking(move);

        sleep(2000);

        Action move2 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToX(PARAMS.MoveOneX - 5)
                .splineToSplineHeading(new Pose2d(PARAMS.MoveTwoX, PARAMS.MoveTwoY,
                        Math.toRadians(PARAMS.MoveTwoHeadingA)), Math.toRadians(PARAMS.MoveTwoHeadingB))
                .build();
        Actions.runBlocking(move2);

        while (opModeIsActive()) {
            telemetry.addLine("Test Done");
            telemetry.update();
            sleep(500);
        }
    }
}
