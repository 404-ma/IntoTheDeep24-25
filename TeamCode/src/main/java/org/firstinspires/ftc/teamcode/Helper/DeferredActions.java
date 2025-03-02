package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import static java.lang.System.currentTimeMillis;

public class DeferredActions {

    public enum DeferredActionType {
        NO_ACTION("None"),
        PLAY_SOUND("Plays Quack"),
        BEAK_OPEN("Open Beak"),
        BEAK_CLOSE("Close Beak"),
        SUPLEX_BUCKET("Suplex to Bucket"),
        SUPLEX_SLIDE("Suplex to Slide"),
        BEAK_DRIVE_SAFE("Beak Drive Safe"),
        BEAK_OPEN_WIDER("Beak Open Wider"),
        BEAK_CLIMB("Beak to Climb Pos"),
        BUCKET_ClIMB("Bucket to Climb Pos"),
        BUCKET_CATCH( "Bucket to Catch Position"),
        CLIMB_MODE_SWITCH("Switch to Climb Mode"),
        DRIVE_MODE_SWITCH("Switch to Drive Mode");

        private final String description;

        DeferredActionType(String description) {
            this.description = description;
        }

        @Override
        public @NonNull String toString() {
            return description;
        }
    }

    private static class DeferredActionEvent {
        public long triggerTime;
        public DeferredActionType action;

        public DeferredActionEvent(long triggerTime, DeferredActionType action) {
            this.triggerTime = triggerTime;
            this.action = action;
        }
    }

    private static volatile List<DeferredActionEvent> deferredActions = new ArrayList<>();
    // Telemetry
    public static DeferredActionType tlmLastAction = DeferredActionType.NO_ACTION;
    public static Date tlmLastActionTimestamp = new Date();

    public static void CreateDeferredAction(long deltaMS, DeferredActionType event) {
        long triggerTime = currentTimeMillis() + deltaMS;
        deferredActions.add(new DeferredActionEvent(triggerTime, event));
    }

    public static void ClearDeferredActions() {
        // --> Clears All deferredActions
        deferredActions.clear();
    }

    public static void CancelDeferredAction(DeferredActionType action) {
        // Remove the specific actions from deferredActions
        deferredActions.removeIf(act -> act.action == action);
    }

    public static boolean HasDeferredActions() {
        return !deferredActions.isEmpty(); // --> checks if deleted
    }

    // Check for Deferred Actions that are Ready to be Processed;
    public static List<DeferredActionType> GetReadyActions() {
        List<DeferredActionType> readyActions = new ArrayList<>();
        List<DeferredActionEvent> removals = new ArrayList<>();


        // Build List of Actions Ready to Execute
        for (DeferredActionEvent act : deferredActions) {
            if (currentTimeMillis() >= act.triggerTime) {
                readyActions.add(act.action);
                removals.add(act);
                tlmLastAction = act.action;
                tlmLastActionTimestamp = new Date();
            }
        }

        // Remove Ready Actions from Deferred list
        deferredActions.removeAll(removals);

        return readyActions;
    }
}