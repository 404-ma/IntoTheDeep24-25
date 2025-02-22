package org.firstinspires.ftc.teamcode.Helper.Beak;

import android.os.SystemClock;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions.DeferredActionType;

@Config
public class newBeak {

    public static class Params {
        //slider
        public double sliderMaxPos = 0.375;
        public double sliderMinPos = 0.0;
        public double sliderPosChange = 0.03;
        public double sliderClimbPos = 0.0;
        public double sliderRetractDelayPosition = 0.2425;

        //hover height is wrong
        //slider needs to be faster
        //fix suplex sequence
        //beak
        public double beakOpenDropPos = 0.62; //for suplex
        public double beakOpenPickupPos = 0.63; //for pick up
        public double beakWideOpen = 0.67; // for wider opening
        public double beakClosePos = 0.47; // closed

        //elbow
        public double elbowPickPos = 0.48;// Pickup Off Mat //0.479
        public double elbowPickAuton = 0.479;
        public double elbowReachPos = 0.49;    // Grabber Extended Drive
        public double elbowSuplexBucketPos = 0.56;    // Suplex in Bucket
        public double elbowReachPosThird = 0.479;    // Grabber Extended Drive

        public double elbowSuplexSlideDumpPos = 0.57; // Suplex to Slide
        public double elbowStartPos = 0.54;    // Drive Position
        public double elbowClimbInit = 0.539;    // Climb Start - Beak Forward
        public double elbowClimbSafePos = 0.575; // Climb - Beak Tucked Down
        public double elbowIncrease = 0.02;
        public double elbowDecrease = -0.011;

        //delays
        public long beakClosedDelay = 50;
        public long beakPickUpDelay = 200;
        // ms Wait for Slider to Retract ot Min Pos before Suplex
        public long suplexSliderRetractDelay = 160;
        // ms Wait for Elbow Suplex to Bucket Before Opening Beak
        public long suplexBucketOpenBeakDelay = 740;
        // ms Wait for Elbow Suplex to Slide Dump Before Opening Beak
        public long suplexSlideDumpOpenBeakDelay = 1050;
        // ms Wait for Sample to Fall in Bucket or Slide
        public long suplexMoveToDrivePositionDelay = 225;
        //ms Until Open Beak Wide When Reaching for Sample
        public long pickupBeakOpenDelay = 100;
        // ms Need for Climb Initialize
        public long climbBeakInitializeDelay = 300;
        public long climbBeakFinalPosDelay = 1000;
    }

    public static Params PARAMS = new Params();

    public static double targetSliderPosition = -1;
    public static double targetBeakPosition = -1;
    public static double targetElbowPosition = -1;

    private final Servo viper;
    private final Servo beak;
    private final Servo elbow;

    public newBeak(@NonNull HardwareMap hardwareMap) {
        viper = hardwareMap.servo.get("viperServo");
        viper.setDirection(Servo.Direction.FORWARD);

        beak = hardwareMap.servo.get("beakServo");
        beak.setDirection(Servo.Direction.FORWARD);

        elbow = hardwareMap.servo.get("elbowServo");
        elbow.setDirection(Servo.Direction.FORWARD);
    }

    //the viper slide
    public void MoveSlider(double newPos) {
        viper.setPosition(newPos);
        targetSliderPosition = newPos;
    }

    public void MoveElbow(double newPos){
        elbow.setPosition(newPos);
        targetElbowPosition = newPos;
    }

    public void MoveBeak(double newPos){
        beak.setPosition(newPos);
        targetBeakPosition = newPos;
    }

    public void IncreaseElbow(){
        targetElbowPosition += PARAMS.elbowIncrease;
        MoveElbow(targetElbowPosition);
    }

    public void  DecreaseElbow(){
        targetElbowPosition += PARAMS.elbowDecrease;
        MoveElbow(targetElbowPosition);
    }


    public void slideClimbPosition() {
        MoveSlider(PARAMS.sliderClimbPos);
    }

    public void JoystickMoveSlide(float position) {
        double sliderPos = Range.clip((targetSliderPosition + (position * PARAMS.sliderPosChange)), PARAMS.sliderMinPos, PARAMS.sliderMaxPos);
        MoveSlider(sliderPos);
    }

    //the servo for beak
    public void closedBeak() {
       MoveBeak(PARAMS.beakClosePos);
    }

    public void openBeak() {
        if (targetElbowPosition < PARAMS.elbowStartPos)
            MoveBeak(PARAMS.beakOpenPickupPos);
        else
            MoveBeak(PARAMS.beakOpenDropPos);
    }

    public void openWideBeak(){
        MoveBeak(PARAMS.beakWideOpen);
    }

    public void ToggleBeak() {
        if (targetBeakPosition == PARAMS.beakClosePos) {
            openBeak();
        } else {
            closedBeak();
        }
    }

    //the servo for elbow
    public void PickUpElbow() {
        // Kill Any Suplex Related Deferred Actions
        DeferredActions.CancelDeferredAction(DeferredActionType.SUPLEX_BUCKET);
        DeferredActions.CancelDeferredAction(DeferredActionType.SUPLEX_SLIDE);
        DeferredActions.CancelDeferredAction(DeferredActionType.BEAK_OPEN);
        DeferredActions.CancelDeferredAction(DeferredActionType.BEAK_DRIVE_SAFE);
        MoveElbow(PARAMS.elbowPickPos);
        DeferredActions.CreateDeferredAction(PARAMS.beakPickUpDelay, DeferredActionType.BEAK_OPEN_WIDER);
    }


    //button back
    public void ElbStart(){
        MoveElbow(PARAMS.elbowStartPos);
        openBeak();
    }


    public void SuplexSampleBucket() {
        if (targetBeakPosition != PARAMS.beakClosePos) {
            closedBeak();
            DeferredActions.CreateDeferredAction(PARAMS.beakClosedDelay, DeferredActionType.SUPLEX_BUCKET);
        } else if (targetSliderPosition >= PARAMS.sliderRetractDelayPosition) {
            MoveSlider(PARAMS.sliderMinPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexSliderRetractDelay, DeferredActionType.SUPLEX_BUCKET);
        } else {
            MoveSlider(PARAMS.sliderMinPos);
            MoveElbow(PARAMS.elbowSuplexBucketPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexBucketOpenBeakDelay, DeferredActionType.BEAK_OPEN);
            long delay = PARAMS.suplexBucketOpenBeakDelay + PARAMS.suplexMoveToDrivePositionDelay;
            DeferredActions.CreateDeferredAction(delay, DeferredActionType.BEAK_DRIVE_SAFE);


        }
    }

    public void SuplexSampleSlideDump() {
        if (targetBeakPosition != PARAMS.beakClosePos) {
            closedBeak();
            DeferredActions.CreateDeferredAction(PARAMS.beakClosedDelay, DeferredActionType.SUPLEX_SLIDE);
        } else if (targetSliderPosition >= PARAMS.sliderRetractDelayPosition) {
            MoveSlider(PARAMS.sliderMinPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexSliderRetractDelay, DeferredActionType.SUPLEX_SLIDE);
        } else {
            MoveSlider(PARAMS.sliderMinPos);
            MoveElbow(PARAMS.elbowSuplexSlideDumpPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexSlideDumpOpenBeakDelay, DeferredActionType.BEAK_OPEN);
            long delay = PARAMS.suplexSlideDumpOpenBeakDelay + PARAMS.suplexMoveToDrivePositionDelay;
            DeferredActions.CreateDeferredAction(delay, DeferredActionType.BEAK_DRIVE_SAFE);
        }
    }


    public void SuplexHold() {
        MoveSlider(PARAMS.sliderMinPos);
        MoveElbow(PARAMS.elbowSuplexSlideDumpPos);
    }


    public void sampleReachElbowPos() {
        // Kill Any Suplex Related Deferred Actions
        DeferredActions.CancelDeferredAction(DeferredActionType.SUPLEX_BUCKET);
        DeferredActions.CancelDeferredAction(DeferredActionType.SUPLEX_SLIDE);
        DeferredActions.CancelDeferredAction(DeferredActionType.BEAK_OPEN);
        DeferredActions.CancelDeferredAction(DeferredActionType.BEAK_DRIVE_SAFE);
        if (targetElbowPosition > PARAMS.elbowStartPos)
            openBeak();
        else
            DeferredActions.CreateDeferredAction( PARAMS.pickupBeakOpenDelay, DeferredActionType.BEAK_OPEN);
        MoveElbow(PARAMS.elbowReachPos);
        openBeak();
    }

    public void toggleElbowSuplex() {
        // star
        if (targetElbowPosition >= PARAMS.elbowStartPos)
            sampleReachElbowPos();
        else
            SuplexSampleBucket();
    }

    public long ClimbInitialize() {
        // Returns Delay Needed for Climb Initialization to Complete
        long delay = ((targetElbowPosition >= PARAMS.elbowClimbInit) ? PARAMS.climbBeakInitializeDelay : 0);
        closedBeak();
        MoveElbow(PARAMS.elbowClimbInit);
        return (delay);
    }

    public long ClimbPositionDelayNeeded() { return PARAMS.climbBeakFinalPosDelay; }

    public void ClimbPositions(){
        closedBeak();
        MoveSlider(PARAMS.sliderMinPos);
        MoveElbow(PARAMS.elbowClimbSafePos);
    }


    /**
     * Autonomous Functions
    */

    public void autonStartPos(){
        MoveSlider(PARAMS.sliderMinPos);
        MoveElbow(PARAMS.elbowStartPos);
        openBeak();
    }

    public Action autonSP(){
        return packet ->{
        MoveSlider(PARAMS.sliderMinPos);
        MoveElbow(PARAMS.elbowStartPos);
        openBeak();
        return false;
        };

    }


    public Action autonReachSamp() {
        return packet -> {
            openBeak();
            MoveElbow(PARAMS.elbowPickAuton);
            SystemClock.sleep(760);
            closedBeak();
            SystemClock.sleep(PARAMS.beakClosedDelay);
            MoveElbow(PARAMS.elbowSuplexBucketPos);
            SystemClock.sleep( PARAMS.suplexBucketOpenBeakDelay);
            openBeak();
            SystemClock.sleep( 680);
            ElbStart();
            return false;
        };
    }

    public Action autonReachOB() {
        return packet -> {
            MoveElbow(PARAMS.elbowPickPos);

            MoveBeak(PARAMS.beakOpenDropPos);
            SystemClock.sleep(PARAMS.beakPickUpDelay);
            MoveBeak(PARAMS.beakWideOpen + 0.03);

            SystemClock.sleep(500);
            return false;
        };
    }

    public Action autonPickupOB() {
        return packet -> {
            MoveElbow(PARAMS.elbowPickPos);

            MoveBeak(PARAMS.beakOpenDropPos);
            SystemClock.sleep(PARAMS.beakPickUpDelay);
            MoveBeak(PARAMS.beakWideOpen + 0.03);


            SystemClock.sleep(500);
            closedBeak();
            SystemClock.sleep(PARAMS.beakClosedDelay);

            MoveElbow(PARAMS.elbowStartPos);
            return false;
        };
    }

    public Action autonPickupToSlide() {
        return packet -> {
            SystemClock.sleep(50); // Delay for Ozer - Let Robot Turn Before Beak Close
            closedBeak();
            SystemClock.sleep(PARAMS.beakClosedDelay);

            MoveElbow(PARAMS.elbowSuplexSlideDumpPos);
            SystemClock.sleep(PARAMS.suplexSlideDumpOpenBeakDelay);
            return false;
        };
    }

    public Action autonSliderExtend(){
        return packet -> {
            MoveSlider(PARAMS.sliderMaxPos);
            SystemClock.sleep(1000);
            return false;
        };
    }

    public Action autonSliderRetract() {
        return packet -> {
            MoveSlider(PARAMS.sliderMinPos);
            SystemClock.sleep(1000);
            return false;
        };
    }

    public Action autonDropSampleToHuman() {
        return packet -> {
            // Drop Sample to Human Player
            MoveElbow(PARAMS.elbowPickPos);
            SystemClock.sleep(500);
            MoveBeak(PARAMS.beakOpenDropPos);
            SystemClock.sleep(PARAMS.beakClosedDelay);

            MoveElbow(PARAMS.elbowStartPos);
            SystemClock.sleep(500);
            return false;
        };
    }


    public Action autonDropToHuman() {
        return packet -> {
            SystemClock.sleep(500);
            openBeak();
            SystemClock.sleep(200);
            ElbStart();
            return false;
        };
    }

    public Action grabAndDrop(){
        return packet -> {
            MoveSlider(PARAMS.sliderMaxPos);
            SystemClock.sleep(1000);
            SystemClock.sleep(600);
            closedBeak();
            SystemClock.sleep(PARAMS.beakClosedDelay);
            MoveElbow(PARAMS.elbowSuplexSlideDumpPos);

            return false;
        };
    }

    public Action autonSuplexToBucket() {
        return packet -> {
            // Close Beak
            if (targetBeakPosition != PARAMS.beakClosePos) {
                closedBeak();
                SystemClock.sleep(PARAMS.beakClosedDelay);
            }

            // Retract Slider
            if (targetSliderPosition >= PARAMS.sliderRetractDelayPosition) {
                MoveSlider(PARAMS.sliderMinPos);
                SystemClock.sleep(PARAMS.suplexSliderRetractDelay);
            } else
                MoveSlider(PARAMS.sliderMinPos);

            // Suplex to Bucket
            MoveElbow(PARAMS.elbowSuplexBucketPos);
            SystemClock.sleep(PARAMS.suplexBucketOpenBeakDelay);
            openBeak();

            // Drive Safe
            SystemClock.sleep(PARAMS.suplexMoveToDrivePositionDelay);
            autonStartPos();
            return false;
        };
    }



}
