package org.firstinspires.ftc.teamcode.Helper.Intake;

/*
   IntakeAction is the wrapper class for all intake related mechanisms
   it handles event scheduling and bus event emitting
 */

import org.firstinspires.ftc.teamcode.Helper.DeferredActions;
import org.firstinspires.ftc.teamcode.Helper.ReactiveState.Reactive;
import org.firstinspires.ftc.teamcode.Helper.ReactiveState.ReactiveState;
import org.firstinspires.ftc.teamcode.Helper.ReactiveState.StateChange;

public class IntakeAction {
    public static class Params {
    }
    public static Params PARAMS = new Params();

    private IntakeRotation intakeRotation;

    @StateChange("handleRotate")
    public ReactiveState<Boolean> isRotated = new ReactiveState<>(false);

    @StateChange("handlePinch")
    public ReactiveState<Boolean> isPinched = new ReactiveState<>(false);

    private void handleRotate() {
        if (this.isRotated.get()) {
            // emit

        } else {
            // emit
        }
    }


    private void handlePinch() {
        if (this.isPinched.get()) {
            // emit a rotation event in 1 second
            DeferredActions.CreateDeferredAction(1000, DeferredActions.DeferredActionType.ROTATE_INTAKE);
        } else {
            // emit a derotation event after 1second
            DeferredActions.CreateDeferredAction(1000, DeferredActions.DeferredActionType.DEROTATE_INTAKE);
        }
    }

    public void TEST_rotation() {
        this.intakeRotation.activateRotation();
    }

    public void TEST_derotate() {
        this.intakeRotation.deactivateRotation();
    }

    public IntakeAction() {
        Reactive.init(this);

        this.intakeRotation = new IntakeRotation(this);

        if (this.intakeRotation.initErrorStatus) {
            // ERR
        }
    }
}
