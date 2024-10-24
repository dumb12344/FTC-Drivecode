package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous test",group="Linear OpMode")
public abstract class Autonomous extends LinearOpMode {

    @Override
    public void runOpMode(){
        waitForStart();
        if(opModeIsActive()){
            telemetry.addData("test","test");
        }
    }
}