package org.firstinspires.ftc.teamcode.Helper.Beak;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DeferredActions;

public class newBeak {

    public static class Params {
        //slider
        public double sliderMaxPos = 0.445;
        public double sliderMinPos = 0.09;

        //beak
        public double beakOpenPos = 0.038;
        public double beakClosePos = 0.65;

        //elbow
        public double elbowPickPos = 0.43;
        public double elbowSuplexPos = 0.52;
        public int times = 0;

        public double beakDelay = 500;


    }

    public static newBeak.Params PARAMS = new newBeak.Params();
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

    public void JoystickMoveSlide(float position) {
        double sliderPos = Range.clip((targetSliderPosition + (position * 0.005)), PARAMS.sliderMinPos, PARAMS.sliderMaxPos);
        MoveSlider(sliderPos);
    }

    //the servo for beak
    public void closedBeak() {
        beak.setPosition(PARAMS.beakClosePos);
    }

    public void openBeak() {
        beak.setPosition(PARAMS.beakOpenPos);
    }

    public void ToggleBeak() {
        PARAMS.times++;
        if (PARAMS.times % 2 == 0) {
            closedBeak();
        } else {
            openBeak();
        }
    }

    //the servo for elbow
    public void PickUpElbow() {
        elbow.setPosition(PARAMS.elbowPickPos);
    }

    public void suplexElbPos() {
        elbow.setPosition(PARAMS.elbowSuplexPos);
    }

    public Action autonReachSamp() {
        return packet -> {
            openBeak();
            PickUpElbow();

            return false;
        };
    }
    public void SuplexSample() {
        if (targetBeakPosition != PARAMS.beakClosePos)  {
            closedBeak();
            DeferredActions.CreateDeferredAction(100, DeferredActions.DeferredActionType.SUPLEX_BEAK);
        } else {
            /*
            MoveArm(PARAMS.armBucketDropPos);
            MoveElbow(PARAMS.elbowBucketDropPos);
            DeferredActions.CreateDeferredAction( (long) PARAMS.beakSuplexOpenDelay, DeferredActions.DeferredActionType.BEAK_OPEN);
            DeferredActions.CreateDeferredAction( (long) PARAMS.beakSuplexDriveDelay, DeferredActions.DeferredActionType.BEAK_DRIVE_SAFE);*/
        }
    }
    public Action autonSuplexSam() {
        return packet -> {
            closedBeak();
            suplexElbPos();
            return false;
        };
    }



}
