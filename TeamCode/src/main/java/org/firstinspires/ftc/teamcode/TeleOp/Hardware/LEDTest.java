package org.firstinspires.ftc.teamcode.TeleOp.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.GamePad;
import org.firstinspires.ftc.teamcode.Helper.LEDColorHelper;


@Config
@Disabled
@TeleOp(name="LED Test", group="Hardware")
public class LEDTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("LED Test");
        telemetry.addData("Version Number", 1.0);
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        GamePad gamePad1Helper = new GamePad(gamepad1);
        LEDColorHelper ledColorHelper = new LEDColorHelper(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();

        while (opModeIsActive()) {
            GamePad.GameplayInputType inpType = gamePad1Helper.WaitForGamepadInput(30);

            switch (inpType) {
                case BUTTON_X:
                    ledColorHelper.setLEDColor(LEDColorHelper.LEDColor.RED);
                    telemetry.addData("LED Color", "Red");
                    break;

                case BUTTON_Y:
                    ledColorHelper.setLEDColor(LEDColorHelper.LEDColor.BLUE);
                    telemetry.addData("LED Color", "Blue");
                    break;

                case BUTTON_B:
                    ledColorHelper.setLEDColor(LEDColorHelper.LEDColor.WHITE);
                    telemetry.addData("LED Color", "White");
                    break;

                case BUTTON_R_BUMPER:
                    ledColorHelper.setLEDColor(LEDColorHelper.LEDColor.AZURE);
                    telemetry.addData("LED Color", "Azure");
                    break;
            }
            telemetry.update();
        }
    }
}




