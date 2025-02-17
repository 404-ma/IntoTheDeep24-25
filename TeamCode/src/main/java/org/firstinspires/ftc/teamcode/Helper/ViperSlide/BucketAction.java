package org.firstinspires.ftc.teamcode.Helper.ViperSlide;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class BucketAction {
    public static class Params {
       // public double bucketStartPos = 0.32;   // Tucked in For Driving
        public double bucketCatchPos = 0.975;  // Catch from Beak
        public double bucketDumpPos = 0.475;    // Dump to Basket
        public double bucketClimbSafePos = 0.0; // Wrap to outside of Viper Slide
        //0.08 --> straight up
    }

    public static Params PARAMS = new Params();

    public static double targetBucketPosition = -1;
    private final Servo bucketServo;


    public BucketAction(@NonNull HardwareMap hdwMap) {
        bucketServo = hdwMap.servo.get("bucketServo");
        bucketServo.setDirection(Servo.Direction.FORWARD);
    }

    private void MoveBucket(double position) {
        bucketServo.setPosition(position);
        targetBucketPosition = position;
    }

    public void StartPosition() {
        MoveBucket(PARAMS.bucketCatchPos);
    }

    public void DumpSample() {
        MoveBucket(PARAMS.bucketDumpPos);
    }

    public void PrepForCatch() {
        MoveBucket(PARAMS.bucketCatchPos);
    }

    public void ToggleBucket() {
        if (targetBucketPosition != PARAMS.bucketDumpPos)
            DumpSample();
        else
            PrepForCatch();
    }

    public void climbPostitions(){ MoveBucket(PARAMS.bucketClimbSafePos); }

    public Action autonDumpSample(){
        return packet ->{
            DumpSample();
          return false;
        };
    }

    public Action autonPrepForCatch(){
        return packet ->{
          PrepForCatch();
          SystemClock.sleep(100);
          return false;
        };
    }

    public Action autonHuman(){
        return packet -> {
            MoveBucket(PARAMS.bucketDumpPos);
            SystemClock.sleep(500);
            return false;
        };
    }

    public Action autonBucketDown(){
        return packet -> {
            MoveBucket(PARAMS.bucketCatchPos);
          return false;
        };
    }
}
