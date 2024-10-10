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
    public static void init(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        armBaseMotor = hardwareMap.get(DcMotor.class, "arm_base");
        jointOneMotor = hardwareMap.get(DcMotor.class, "joint_one");
    }
}
