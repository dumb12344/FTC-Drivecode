package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class util {
    public static DcMotor frontLeftDrive = null;
    public static DcMotor frontRightDrive = null;
    public static DcMotor backLeftDrive = null;
    public static DcMotor backRightDrive = null;
    public static DcMotor armBaseMotor = null;
    public static DcMotor jointOneMotor = null;
    /**
     * Initializes motors (and eventually servos)
     * @param hardwareMap
     * @see TeleOpMode
     * @see AutonomousOpMode
     */
    public static void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        armBaseMotor = hardwareMap.get(DcMotor.class, "arm_base");
        //jointOneMotor = hardwareMap.get(DcMotor.class, "joint_one");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        armBaseMotor.setDirection(DcMotor.Direction.FORWARD); // REV Robotics 20:1 HD Hex Motor
        //jointOneMotor.setDirection(DcMotor.Direction.FORWARD); // REV Robotics Core Hex Motor

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBaseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //jointOneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
