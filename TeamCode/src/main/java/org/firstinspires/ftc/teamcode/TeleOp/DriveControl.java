package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.Beak.newBeak;
import org.firstinspires.ftc.teamcode.Helper.HangAction;
import org.firstinspires.ftc.teamcode.Helper.LEDColorHelper;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.BucketAction;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ClawAction;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions.DeferredActionType;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.GamePad;
import org.firstinspires.ftc.teamcode.Helper.ViperSlide.ViperAction;



import java.util.List;
import java.util.Locale;

@Config
@TeleOp(name = "Driver Control", group = "Competition!!")
public class DriveControl extends LinearOpMode {
    private static final String version = "2.7";

    private ViperAction viperAction;
    private BucketAction bucketAction;
    private ClawAction clawAction;
    private HangAction hangAction;
    private newBeak beakAction;
    private LEDColorHelper colorful;
    private GamePad gpIn1;
    private GamePad gpIn2;
    private DrivetrainV2 drvTrain;

    private boolean thirdScheme = false;
    private boolean schemeTransition = false;
    private double speedMultiplier = 1;
    private double lastSpeed = 1;


    @Override
    public void runOpMode() {
        telemetry.clear();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Driver Control");
        telemetry.addData("Version Number", version);
        telemetry.addLine();

        int initRes = initialize();
        if (initRes == 0)
            telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        waitForStart();
        if (isStopRequested() || (initRes == 1)) {
            return;
        }

        telemetry.clear();
        colorful.setLEDColor(LEDColorHelper.LEDColor.AZURE);
        beakAction.autonStartPos();
        bucketAction.StartPosition();

        gpIn1 = new GamePad(gamepad1, false);
        gpIn2 = new GamePad(gamepad2, false);
        drvTrain = new DrivetrainV2(hardwareMap);

        while (opModeIsActive()) {
            update_telemetry(gpIn1, gpIn2); // Update LED and DriverStation
            padOne();   // Process Gamepad 1
            padTwo();   // Process Gamepad 2
           ProcessDeferredActions(); // Deferred Actions
        }
    }


    public void padOne(){
        GamePad.GameplayInputType inpType1 = gpIn1.WaitForGamepadInput(30);
        if (!thirdScheme) {
            switch (inpType1) {
                case DPAD_DOWN:
                    beakAction.DecreaseElbow();
                    break;
                case DPAD_UP:
                    beakAction.IncreaseElbow();
                    break;
                case DPAD_LEFT:
                    beakAction.SuplexHold();
                    break;
                case DPAD_RIGHT:
                    beakAction.PickUpElbow();
                    break;
                case BUTTON_R_BUMPER:
                    beakAction.ToggleBeak();
                    break;
                case BUTTON_L_BUMPER:
                    beakAction.toggleElbowSuplex();
                    break;
                case LEFT_STICK_BUTTON_ON:
                    if (speedMultiplier != 1) {
                        lastSpeed = speedMultiplier;
                        speedMultiplier = 1;
                    }
                    break;
                case LEFT_STICK_BUTTON_OFF:
                    if (lastSpeed != 1) {
                        speedMultiplier = lastSpeed;
                        lastSpeed = 1;
                    }
                    break;
                case BUTTON_X:
                    speedMultiplier = 0.75;
                    break;
                case BUTTON_B:
                    speedMultiplier = 0.5;
                    break;
                case BUTTON_A:
                    speedMultiplier = 0.25;
                    break;
                case BUTTON_Y:
                    speedMultiplier = 1;
                    break;
                case BUTTON_BACK:
                    bucketAction.DumpSample();
                    beakAction.SuplexSampleSlideDump();
                    break;
                case RIGHT_TRIGGER:
                    beakAction.JoystickMoveSlide(gamepad1.right_trigger);
                    break;
                case LEFT_TRIGGER:
                    beakAction.JoystickMoveSlide(-gamepad1.left_trigger);
                    break;
                case JOYSTICK:
                    drvTrain.setDriveVectorFromJoystick(gamepad1.left_stick_x * (float) speedMultiplier,
                            gamepad1.right_stick_x * (float) speedMultiplier,
                            gamepad1.left_stick_y * (float) speedMultiplier, false);
                    break;
            }
        } else {
            switch (inpType1) {
                case LEFT_TRIGGER:
                    hangAction.moveStage1Motors(-gamepad1.left_trigger);
                    break;
                case RIGHT_TRIGGER:
                    hangAction.moveStage1Motors(gamepad1.right_trigger);
                    break;
                case JOYSTICK:
                    drvTrain.setDriveVectorFromJoystick(gamepad1.left_stick_x * (float) speedMultiplier,
                            gamepad1.right_stick_x * (float) speedMultiplier,
                            gamepad1.left_stick_y * (float) speedMultiplier, false);
                    break;
            }
        }
    }

    public void padTwo(){
        GamePad.GameplayInputType inpType2 = gpIn2.WaitForGamepadInput(30);
        if (!thirdScheme) {
            switch (inpType2) {
                case BUTTON_L_BUMPER:
                    bucketAction.ToggleBucket();
                    break;
                case BUTTON_R_BUMPER:
                    bucketAction.SampleHoldPosition();
                    break;
                case LEFT_TRIGGER:
                    viperAction.moveWithPower(-gamepad2.left_trigger);
                    break;
                case RIGHT_TRIGGER:
                    viperAction.moveWithPower(gamepad2.right_trigger);
                    break;
                case DPAD_LEFT:
                    viperAction.perfMoveForSub();
                    break;
                case DPAD_RIGHT:
                    viperAction.perfPlaceOnSub();
                    break;
                case DPAD_UP:
                    viperAction.moveToHighBasket();
                    break;
                case DPAD_DOWN:
                    viperAction.moveToLowBasket();
                    break;
                case BUTTON_B:
                    clawAction.OpenGrip();
                    break;
                case BUTTON_X:
                    clawAction.CloseGrip();
                    break;
                case BUTTON_A:
                    viperAction.HoldPosition();
                    break;
                case RIGHT_STICK_BUTTON_ON:
                    viperAction.resetEncoders();
                    break;
                case JOYSTICK:
                    hangAction.moveStage1Motors(-gamepad2.left_stick_y);
                    hangAction.moveStage2Motor(-gamepad2.right_stick_y);
                    break;
                case BUTTON_BACK:
                    // Reconfigure for Climb
                    schemeTransition = true;
                    DeferredActions.ClearDeferredActions();
                    clawAction.CloseGrip();
                    // Compute Deferred Action Delays Based on Current Positions
                    long beakClimbInitCompletionDelay = beakAction.ClimbInitialize();
                    long bucketClimbCompletionDelay = bucketAction.ClimbDelayNeeded() + beakClimbInitCompletionDelay;
                    long climbTransitionCompletionDelay = beakAction.ClimbPositionDelayNeeded() + bucketClimbCompletionDelay;
                    // Queue Deferred Action for Mode Switch
                    DeferredActions.CreateDeferredAction(beakClimbInitCompletionDelay, DeferredActionType.BUCKET_ClIMB);
                    DeferredActions.CreateDeferredAction(bucketClimbCompletionDelay, DeferredActionType.BEAK_CLIMB);
                    DeferredActions.CreateDeferredAction(climbTransitionCompletionDelay, DeferredActionType.CLIMB_MODE_SWITCH);
                    break;
            }
        } else {
            switch (inpType2) {
                case JOYSTICK:
                    hangAction.moveStage1Motors(-gamepad2.left_stick_y);
                    hangAction.moveStage2Motor(-gamepad2.right_stick_y);
                    break;
                case BUTTON_X:
                    hangAction.holdStage2Position();
                    break;
                case DPAD_UP:
                    hangAction.grappleFlipUp();
                    break;
                case DPAD_RIGHT:
                    hangAction.grappleForward();
                    break;
                case DPAD_LEFT:
                    hangAction.grappleBackward();
                    break;
                case DPAD_DOWN:
                    hangAction.grappleStartPosition();
                    break;
                case BUTTON_BACK:
                    // Reconfigure Back to Drive Mode
                    schemeTransition = true;
                    beakAction.autonStartPos();
                    // Allow Time for Beak Transition to Start Position
                    DeferredActions.CreateDeferredAction(500, DeferredActionType.BUCKET_CATCH);
                    DeferredActions.CreateDeferredAction(750, DeferredActionType.DRIVE_MODE_SWITCH);
                    break;
            }
        }
    }

    // Deferred Actions
    public void ProcessDeferredActions(){
        List<DeferredActionType> action = DeferredActions.GetReadyActions();

        for(DeferredActionType actionType: action){
            switch(actionType){
                case BEAK_OPEN:
                    beakAction.openBeak();
                    break;

                case BEAK_CLOSE:
                    beakAction.closedBeak();
                    break;

                case SUPLEX_BUCKET:
                    beakAction.SuplexSampleBucket();
                    break;

                case SUPLEX_SLIDE:
                    beakAction.SuplexSampleSlideDump();
                    break;

                case BEAK_DRIVE_SAFE:
                    beakAction.ElbStart();
                    break;

                case BEAK_OPEN_WIDER:
                    beakAction.openWideBeak();
                    break;

                case BEAK_CLIMB:
                    beakAction.ClimbPositions();
                    break;

                case BUCKET_ClIMB:
                    bucketAction.ClimbPositions();
                    break;

                case BUCKET_CATCH:
                    bucketAction.PrepForCatch();
                    break;

                case CLIMB_MODE_SWITCH:
                    thirdScheme = true;
                    schemeTransition = false;
                    break;

                case DRIVE_MODE_SWITCH:
                    thirdScheme = false;
                    schemeTransition = false;
                    break;

                default:
                    telemetry.addLine("ERROR - Unsupported Deferred Action");
                    break;
            }
        }
    }

    private int initialize() {
        try {
            beakAction = new newBeak(hardwareMap);
            viperAction = new ViperAction(hardwareMap);
            hangAction = new HangAction (hardwareMap);
            bucketAction = new BucketAction(hardwareMap);
            clawAction = new ClawAction(hardwareMap);
            colorful = new LEDColorHelper(hardwareMap);
            return 0;
        } catch(Exception e) {
            telemetry.addLine("*** INITIALIZATION ERROR ***");
            telemetry.addLine();
            telemetry.addLine().addData("Error", e.toString());
            telemetry.update();
            return 1;
        }
    }

    private void update_telemetry(GamePad gpi1, GamePad gpi2) {

        // Update Status LED
        if (thirdScheme) {
            telemetry.addLine("     C L I M B  Mode");
            colorful.setLEDColor(LEDColorHelper.LEDColor.ORANGE);
        } else if (schemeTransition) {
            colorful.setLEDColor(LEDColorHelper.LEDColor.VIOLET);
            telemetry.addLine("    *** CHANGING MODE ***");
        } else {
            telemetry.addLine("     D R I V E  Mode");
            colorful.setLEDColor(LEDColorHelper.LEDColor.AZURE);
        }
        telemetry.addLine();

        // Update Driver Station
        telemetry.addLine("Gamepad #1");
        String inpTime1 = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi1.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData("GP1 Time", inpTime1);
        telemetry.addLine().addData("GP1 Cnt", gpi1.getTelemetry_InputCount());
        telemetry.addLine().addData("GP1 Input", gpi1.getTelemetry_InputLastType().toString());

        telemetry.addLine();
        telemetry.addLine("Gamepad #2");
        String inpTime2 = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(gpi2.getTelemetry_InputLastTimestamp());
        telemetry.addLine().addData("GP 2 Time", inpTime2);
        telemetry.addLine().addData("GP2 Cnt", gpi2.getTelemetry_InputCount());
        telemetry.addLine().addData("GP2 Input", gpi2.getTelemetry_InputLastType().toString());

        telemetry.addLine();
        telemetry.addLine("Deferred Actions");
        String actTime = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(DeferredActions.tlmLastActionTimestamp);
        telemetry.addLine().addData("Time", actTime);
        telemetry.addLine().addData("Action", DeferredActions.tlmLastAction.toString());

        telemetry.update();
    }
}

