package org.firstinspires.ftc.teamcode.Helper.Beak;

import android.os.SystemClock;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions;

@Config
public class newBeak {

    public static class Params {
        //slider
        public double sliderMaxPos = 0.440;
        public double sliderMinPos = 0.045;
        public double sliderPosChange = 0.03;
        public double sliderRetractDelayPosition = 0.2425;

        //hover height is wrong
        //slider needs to be faster
        //fix suplex sequence
        //beak
        public double beakOpenDropPos = 0.62; //for suplex
        public double beakOpenPickupPos = 0.63; //for pick up
        public double beakWideOpen = 0.66; // for wider opening
        public double beakClosePos = 0.47; // closed

        //elbow
        public double elbowPickPos = 0.475;     // Pickup Off Mat
        public double elbowReachPos = 0.49;    // Grabber Extended Drive
        public double elbowSuplexBucketPos = 0.56;    // Suplex in Bucket
        public double elbowSuplexSlideDumpPos = 0.57; // Suplex to Slide
        public double elbowStartPos = 0.541;    // Drive Position
        public double elbowClimbInit = 0.540;    // Climb Start - Beak Forward
        public double elbowClimbSafePos = 0.575; // Climb - Beak Tucked Down

        //delays
        public long beakClosedDelay = 50;
        public long beakPickUpDelay = 200;
        // ms Wait for Slider to Retract ot Min Pos before Suplex
        public long suplexSliderRetractDelay = 160;
        // ms Wait for Elbow Suplex to Bucket Before Opening Beak
        public long suplexBucketOpenBeakDelay = 825;
        // ms Wait for Elbow Suplex to Slide Dump Before Opening Beak
        public long suplexSlideDumpOpenBeakDelay = 1050;
        // ms Wait for Sample to Fall in Bucket or Slide
        public long suplexMoveToDrivePositionDelay = 225;
        //ms Until Open Beak Wide When Reaching for Sample
        public long pickupBeakOpenDelay = 100;
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
        targetElbowPosition += 0.01;
        MoveElbow(targetElbowPosition);
    }

    public void  DecreaseElbow(){
        targetElbowPosition -= 0.01;
        MoveElbow(targetElbowPosition);
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
        DeferredActions.ClearDeferredActions();   // Kill Any Suplex Deferred Actions
        MoveElbow(PARAMS.elbowPickPos);
        DeferredActions.CreateDeferredAction(PARAMS.beakPickUpDelay, DeferredActions.DeferredActionType.BEAK_OPEN_WIDER);
    }


    //button back
    public void ElbStart(){
        MoveElbow(PARAMS.elbowStartPos);
        openBeak();
    }


    public void SuplexSampleBucket() {
        if (targetBeakPosition != PARAMS.beakClosePos) {
            closedBeak();
            DeferredActions.CreateDeferredAction(PARAMS.beakClosedDelay, DeferredActions.DeferredActionType.SUPLEX_BUCKET);
        } else if (targetSliderPosition >= PARAMS.sliderRetractDelayPosition) {
            MoveSlider(PARAMS.sliderMinPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexSliderRetractDelay, DeferredActions.DeferredActionType.SUPLEX_BUCKET);
        } else {
            MoveSlider(PARAMS.sliderMinPos);
            MoveElbow(PARAMS.elbowSuplexBucketPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexBucketOpenBeakDelay, DeferredActions.DeferredActionType.BEAK_OPEN);
            long delay = PARAMS.suplexBucketOpenBeakDelay + PARAMS.suplexMoveToDrivePositionDelay;
            DeferredActions.CreateDeferredAction(delay, DeferredActions.DeferredActionType.BEAK_DRIVE_SAFE);


        }
    }

    public void SuplexSampleSlideDump() {
        if (targetBeakPosition != PARAMS.beakClosePos) {
            closedBeak();
            DeferredActions.CreateDeferredAction(PARAMS.beakClosedDelay, DeferredActions.DeferredActionType.SUPLEX_SLIDE);
        } else if (targetSliderPosition >= PARAMS.sliderRetractDelayPosition) {
            MoveSlider(PARAMS.sliderMinPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexSliderRetractDelay, DeferredActions.DeferredActionType.SUPLEX_SLIDE);
        } else {
            MoveSlider(PARAMS.sliderMinPos);
            MoveElbow(PARAMS.elbowSuplexSlideDumpPos);
            DeferredActions.CreateDeferredAction(PARAMS.suplexSlideDumpOpenBeakDelay, DeferredActions.DeferredActionType.BEAK_OPEN);
            long delay = PARAMS.suplexSlideDumpOpenBeakDelay + PARAMS.suplexMoveToDrivePositionDelay;
            DeferredActions.CreateDeferredAction(delay, DeferredActions.DeferredActionType.BEAK_DRIVE_SAFE);
        }
    }


    public void SuplexHold() {
        MoveSlider(PARAMS.sliderMinPos);
        MoveElbow(PARAMS.elbowSuplexSlideDumpPos);
    }


    public void sampleReachElbowPos() {
        DeferredActions.ClearDeferredActions();   // Kill Any Suplex Deferred Actions
        if (targetElbowPosition > PARAMS.elbowStartPos)
            openBeak();
        else
            DeferredActions.CreateDeferredAction( (long) PARAMS.pickupBeakOpenDelay, DeferredActions.DeferredActionType.BEAK_OPEN);
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

    public void ClimbInitialize() {
        closedBeak();
        MoveElbow(PARAMS.elbowClimbInit);
    }

    public void ClimbPostitions(){
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


    public Action autonReachSamp() {
        return packet -> {
            openBeak();
            PickUpElbow();
            SystemClock.sleep(1000);
            closedBeak();
            SystemClock.sleep(PARAMS.beakClosedDelay);
            MoveElbow(PARAMS.elbowSuplexBucketPos);
            SystemClock.sleep( PARAMS.suplexBucketOpenBeakDelay);
            openBeak();
            SystemClock.sleep( PARAMS.suplexBucketOpenBeakDelay + PARAMS.beakClosedDelay);
            ElbStart();
            return false;
        };
    }


    public Action autonReachOB() {
        return packet -> {
            openBeak();
            PickUpElbow();
            SystemClock.sleep(600);
            closedBeak();
            SystemClock.sleep(PARAMS.beakClosedDelay);
            MoveElbow(PARAMS.elbowSuplexSlideDumpPos);

            return false;
        };
    }



    public Action autonDropToHuman() {
        return packet -> {
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
            PickUpElbow();
            SystemClock.sleep(600);
            closedBeak();
            SystemClock.sleep(500);
            ElbStart();
            SystemClock.sleep(500);
            MoveSlider(PARAMS.sliderMinPos);
            SystemClock.sleep(1000);
            MoveElbow(PARAMS.elbowSuplexSlideDumpPos);
            SystemClock.sleep(1000);
            openBeak();
            SystemClock.sleep(500);
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
