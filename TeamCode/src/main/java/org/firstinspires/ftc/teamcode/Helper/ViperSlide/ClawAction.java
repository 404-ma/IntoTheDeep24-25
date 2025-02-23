package org.firstinspires.ftc.teamcode.Helper.ViperSlide;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawAction {
    public static class Params {
        public boolean gripServoReverse = false;
        public double gripOpenPos = 0.44;
        public double gripClosedPos = 0.15;
    }

    public static Params PARAMS = new Params();

    public static double targetGripPosition = -1;
    private final Servo clawServo;

    public ClawAction(@NonNull HardwareMap hdwMap) {
        clawServo = hdwMap.servo.get("clawServo");
        clawServo.setDirection((PARAMS.gripServoReverse) ?
                Servo.Direction.REVERSE : Servo.Direction.FORWARD);

    }

    private void MoveGrip(double position) {
        clawServo.setPosition(position);
        targetGripPosition = position;
    }

    public Action closeGripAuton(){
        return packet ->{
          CloseGrip();
          return false;
        };
    }
    public void CloseGrip(){
        MoveGrip(PARAMS.gripClosedPos);
    }

    public void OpenGrip(){
        MoveGrip(PARAMS.gripOpenPos);
    }

    public void ToggleGrip() {
        if (targetGripPosition == PARAMS.gripOpenPos)
            CloseGrip();
        else
            OpenGrip();
    }

    /*
     * Autonomous Claw Movements
     */
    public Action placeOnSub () {
        return packet -> {
            MoveGrip(PARAMS.gripOpenPos);
            return false;
        };
    }

    public Action grabFromHuman () {
        return packet -> {
            MoveGrip(PARAMS.gripClosedPos);
            SystemClock.sleep(250);
            return false;
        };
    }

}
