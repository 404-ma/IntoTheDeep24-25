package org.firstinspires.ftc.teamcode.Helper;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class ViperSlide {
    public static class Params {
        public double viperSpeed = 0.7;
        public int viperMotorMaxPositionRelative = 15900;  //
    }

    public static Params PARAMS = new Params();
    public DcMotorEx viperMotor;

    public ViperSlide(@NonNull HardwareMap hdwMap) {
        viperMotor = hdwMap.get(DcMotorEx.class, "viperMotor");
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void moveViperToPosition(int position) {
        int checkedPosition = Range.clip(position, 0, PARAMS.viperMotorMaxPositionRelative);
        viperMotor.setTargetPosition(checkedPosition);
        viperMotor.setPower(PARAMS.viperSpeed);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void moveViperToZero(){
        viperMotor.setTargetPosition(0);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

   public void moveViperWithPower(double power, boolean override) {
        viperMotor.getMode();
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setDirection(DcMotor.Direction.REVERSE);
        if (!override) {
            int viperPosition = viperMotor.getCurrentPosition();

            if (power > 0) {
                if (viperPosition >= PARAMS.viperMotorMaxPositionRelative)
                    power = 0;
                else if (viperPosition <= (PARAMS.viperMotorMaxPositionRelative * 0.95))
                    power = Math.min(power, 0.5);

            } else {
                if (viperPosition <= 0)
                    power = 0;
                else if (viperPosition >= (PARAMS.viperMotorMaxPositionRelative * 0.002))
                    power = Math.max(power, -0.5);
            }
            viperMotor.setPower(Range.clip(power, -1, 1));
        }

    }

    public Action moveToPos(int pos, double power){
        return packet -> {
            viperMotor.setTargetPosition(pos);
            viperMotor.setPower(power);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        };

    }

}
