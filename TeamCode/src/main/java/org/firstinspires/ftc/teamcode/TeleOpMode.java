package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Drive", group="Iterative OpMode")
public class TeleOpMode extends OpMode
{
    //TODO : fix joint / arm
    //

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    util core = new util();
    private double movementSpeedMultiplier = 1.0;
    private double armBasePower = 0;
    private double jointOnePower = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        core.init(hardwareMap);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        // Check if precision mode is enabled
        movementSpeedMultiplier = gamepad1.left_bumper ? 0.5 : 1;

        // Calculate the mecanum drive values
        double drive = -gamepad1.left_stick_y * movementSpeedMultiplier;
        double strafe = gamepad1.left_stick_x * 0.5 * movementSpeedMultiplier; // Reduce strafing speed to half
        double turn = gamepad1.right_stick_x * movementSpeedMultiplier;
        // Calculate the motor powers
        frontLeftPower = drive + strafe + turn;
        frontRightPower = drive - strafe - turn;
        backLeftPower = drive - strafe + turn;
        backRightPower = drive + strafe - turn;

        // Clip the motor powers to ensure they are within the valid range
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower = Range.clip(backRightPower, -1.0, 1.0);

        // Send calculated power to wheels
        core.frontLeftDrive.setPower(frontLeftPower);
        core.frontRightDrive.setPower(frontRightPower);
        core.backLeftDrive.setPower(backLeftPower);
        core.backRightDrive.setPower(backRightPower);

        // Control the arm using the D-pad up and down buttons
        if (gamepad1.dpad_up) {
            armBasePower = 0.5;
        } else if (gamepad1.dpad_down) {
            armBasePower = -0.5;
        } else {
            armBasePower = 0;
        }

        // Adjust the arm base power to account for the 20:1 gear ratio
        armBasePower *= 0.05; // Adjusted to 0.05 to prevent motor from burning out

        // Send calculated power to arm motors
        core.armBaseMotor.setPower(armBasePower);
        core.jointOneMotor.setPower(jointOnePower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Arm Base Power", armBasePower);
        telemetry.addData("Joint One Power", jointOnePower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}