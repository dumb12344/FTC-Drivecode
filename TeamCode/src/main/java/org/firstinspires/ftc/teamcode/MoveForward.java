package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous()
public class MoveForward extends LinearOpMode {

    @Override
    public void runOpMode() {
        util core = new util();
        core.init(hardwareMap);
        telemetry.addLine("Waiting for start");
        telemetry.addLine("Make sure you stop after because I make the wheels stop in this code");
        updateTelemetry(telemetry);
        waitForStart();
        while(opModeIsActive()){
            core.frontLeftDrive.setPower(1.0);
            core.frontRightDrive.setPower(1.0);
            core.backLeftDrive.setPower(1.0);
            core.backRightDrive.setPower(1.0);
        }
    }
}
