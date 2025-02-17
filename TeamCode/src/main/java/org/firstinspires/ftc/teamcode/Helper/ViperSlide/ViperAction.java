package org.firstinspires.ftc.teamcode.Helper.ViperSlide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import android.os.SystemClock;

import androidx.annotation.NonNull;

@Config
public class ViperAction {
    public static class Params {
        public boolean viperMotorReverse = true;
        public int  viperHighBasketPos = 2880;  // High Basket
        public double viperLowBasketPos = 1050;   // Low Basket (Approx 38% of High Basket)
        public double viperCatchPoint = 0;        // Catch Point for Sample
        public double viperMotorSpeed = 0.9;
        public double viperMaxPos = 2900;
        public double viperPowerLimitPos = 2800;

        public double clawLow = 387;
        public double clawLowHang = 0;
        public double clawHigh = 1850;
        public double clawHighHang = 1450;
        public double clawWall = 14;

        public double dumpLowBasketDelay = 1000 ;    //ms To Wait for Dump
        public double dumpHighBasketDelay = 3250;  //ms To Wait for Dump
        public int lowerBasketPosition = 1000;
        public double dumpDelay = 1000;
        public int autonReset = 100;
        public double power = 0;
        public boolean hang = false;
    }

    public static Params PARAMS = new Params();
    private final DcMotor viperMotor;
    private BucketAction bucketAction;
    private ClawAction clawAction;

    public ViperAction(@NonNull HardwareMap hdwMap) {
        viperMotor = hdwMap.get(DcMotor.class, "viperMotor");
        viperMotor.setDirection((PARAMS.viperMotorReverse)? DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketAction = new BucketAction(hdwMap);
        clawAction = new ClawAction(hdwMap);
    }


    private void moveToPosition(int targetPosition) {
        viperMotor.setTargetPosition(targetPosition);

        if(PARAMS.hang == false){
            viperMotor.setPower(PARAMS.viperMotorSpeed);}
        else {
            viperMotor.setPower(0.5);
            PARAMS.hang = false;
        }
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveWithPower(double throttle) {
        PARAMS.power = throttle;
        if (PARAMS.power > 0) {
            if (viperMotor.getCurrentPosition() >= PARAMS.viperMaxPos)
                PARAMS.power = 0;
            else if (viperMotor.getCurrentPosition() >= PARAMS.viperPowerLimitPos)
                PARAMS.power = Math.min(PARAMS.power, 0.25);
        } else if ((PARAMS.power < 0) && (viperMotor.getCurrentPosition() <= 400)) {
            PARAMS.power = Math.max(PARAMS.power, -0.25);
        }


        if (viperMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
            viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setPower(PARAMS.power);
    }

    public void HoldPosition() {
        moveToPosition(viperMotor.getCurrentPosition());
    }

    public void resetEncoders() {
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public int getCurrentPosition() {
        return viperMotor.getCurrentPosition();
    }


    public void moveToHighBasket() {
        moveToPosition((int) PARAMS.viperHighBasketPos);
    }


    public void moveToLowBasket() {

        moveToPosition((int) PARAMS.viperLowBasketPos);
    }



    /*
     * Autonomous Viper Movements
     */
    public Action dumpSampleLowBasket() {
        return packet -> {
            moveToLowBasket();
            SystemClock.sleep((long) PARAMS.dumpLowBasketDelay);
            bucketAction.DumpSample();
            SystemClock.sleep((long)PARAMS.dumpDelay);

            return false;
        };
    }

    public Action dumpSampleHighBasket() {
        return packet -> {
            moveToHighBasket();
            SystemClock.sleep((long) PARAMS.dumpHighBasketDelay);
            bucketAction.DumpSample();
            SystemClock.sleep((long)PARAMS.dumpDelay);
            return false;
        };
    }

    /*
    Claw + Viper
     */
    public void moveForSub () {PARAMS.hang = true; moveToPosition((int) PARAMS.clawLow);}
    public void placeOnSub () {PARAMS.hang = true; moveToPosition((int) PARAMS.clawLowHang);}
    public void clawHuman () {PARAMS.hang = true; moveToPosition((int) PARAMS.clawWall);}
    public void perfMoveForSub () {PARAMS.hang = true; moveToPosition((int) PARAMS.clawHigh);}
    public void perfPlaceOnSub () {PARAMS.hang = true; moveToPosition((int) PARAMS.clawHighHang);}

    /*
    Claw + Viper Auto
     */

    public Action clawDropOnSub () {
        return packet -> {
            placeOnSub();
            SystemClock.sleep(250);
          return false;
        };
    }

    public Action clawHumanGrab () {
        return packet -> {
            clawHuman();
            SystemClock.sleep(250);
            return false;
        };
    }

    public Action beforeDropOff () {
        return packet -> {
            moveForSub();
            SystemClock.sleep(250);
            return false;
        };
    }
    public Action perfClawDropOnSub () {
        return packet -> {
            perfPlaceOnSub();
            SystemClock.sleep(900);
            return false;
        };
    }

    public Action perfBeforeDropOff() {
        return packet -> {
            perfMoveForSub();
            SystemClock.sleep(900);
            return false;
        };
    }

    public Action autonLowerBucket(){
        return packet -> {
            moveToPosition(PARAMS.viperHighBasketPos);
          return false;
        };
    }

    public Action autonReset(){
        return packet -> {
            moveToPosition(PARAMS.autonReset);
            return false;
        };
    }
}


