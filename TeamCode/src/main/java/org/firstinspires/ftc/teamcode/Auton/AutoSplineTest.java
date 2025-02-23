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
@Autonomous(name = "Spline Test", group = "Test")
public class AutoSplineTest extends LinearOpMode {
    public static class Params {
        public double MoveOneX = 30;
        public double MoveOneY = 30;
        public double MoveOneHeadingA = Math.toRadians(180);
        public double MoveOneHeadingB = Math.toRadians(0);

        public double MoveTwoX = 11;
        public double MoveTwoY = 11;
        public double MoveTwoHeadingA = Math.toRadians(-90);
        public double MoveTwoHeadingB = Math.toRadians(0);
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

        sleep(1000);

        Action move2 = drive.actionBuilder(drive.pose)
                .setReversed(true)
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
