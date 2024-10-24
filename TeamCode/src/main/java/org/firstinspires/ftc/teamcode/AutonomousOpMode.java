package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous test",group="Linear OpMode")
public class AutonomousOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){
        waitForStart();
        if(opModeIsActive()){
            telemetry.addData("test","test");
        }
    }
}