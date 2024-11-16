package org.firstinspires.ftc.teamcode.Helper.Intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.DependencyInjection.Inject;
import org.firstinspires.ftc.teamcode.Helper.DependencyInjection.Injectable;

public class Pinch extends Injectable {
    public static class Params {
        public double grabOpenPos = 0.600;
        public double grabClosedPos = 0.355;
    }

    public boolean initErrorStatus = false;
    public String initError = "";

    public static Params PARAMS = new Params();

    private IntakeAction intakeAction;

    public double tlmGrabPosition = -1;

    @Inject("hdwMap")
    private HardwareMap hdwMap;

    @Inject("pinchServoName")
    public String servoName;

    private Servo grabber;

    public Pinch(IntakeAction intakeAction){
         super();

        this.intakeAction = intakeAction;

        try {
            grabber = hdwMap.servo.get(servoName);
            grabber.setDirection(Servo.Direction.FORWARD);
        } catch(Exception e) {
            this.initErrorStatus = true;
            this.initError = e.toString();
        }
    }

    public void MovePincher(double position) {
        grabber.setPosition(position);
        tlmGrabPosition = position;
    }

    public void AutonomousStart () {
        MovePincher(PARAMS.grabOpenPos);
    }

    /*
     * Driver Claw Movements
     */

    public void closeGrip(){
        MovePincher(PARAMS.grabClosedPos);

        this.intakeAction.isPinched.set(true);
    }

    public void openGrip(){
        MovePincher(PARAMS.grabOpenPos);

        this.intakeAction.isPinched.set(false);
    }

}
