package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Code for TeleOp mode, where you use the controller
 * @see util
 * @see OpMode
 * @see TeleOp
 */
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

private boolean alignMode = false;
    private static boolean targetBlue = true; // Default to blue target
    private OpenCvCamera camera;

    // Proportional control constants
    private static final double ROTATION_KP = 0.005; // Adjust as needed
    private static final double STRAFE_KP = 0.005;   // Adjust as needed

    // Maximum power limits to prevent overcorrection
    private static final double MAX_ROTATION_POWER = 0.3;
    private static final double MAX_STRAFE_POWER = 0.3;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        core.init(hardwareMap);

        // Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "cam_1"), cameraMonitorViewId);
        camera.setPipeline(new GamePiecePipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Error opening camera: " + errorCode);
            }
        });

        telemetry.addData("Camera", "Opened and streaming");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
// Toggle alignment mode with 'A' button and set color target
        if (gamepad1.a) {
            alignMode = true;
        } else {
            alignMode = false;
        }

        if (gamepad1.b) {
            targetBlue = !targetBlue; // Toggle between blue and red
            // Debounce toggle if needed
            try {
                Thread.sleep(200); // 200 ms debounce
            } catch (InterruptedException e) {
                // Handle exception
            }
        }

        if (alignMode) {
            // Vision alignment mode
            double centerX = GamePiecePipeline.centerX;
            double frameCenterX = 160; // Half of the 320px frame width
            double errorX = centerX - frameCenterX;

            // Proportional control for rotation
            double rotationPower = Range.clip(errorX * ROTATION_KP, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);

            // Proportional control for strafing
            // Assuming that if the block is off-center, we need to strafe towards it
            double strafePower = Range.clip(-errorX * STRAFE_KP, -MAX_STRAFE_POWER, MAX_STRAFE_POWER);

            // Combine rotation and strafing
            frontLeftPower = rotationPower + strafePower;
            frontRightPower = -rotationPower - strafePower;
            backLeftPower = rotationPower - strafePower;
            backRightPower = -rotationPower + strafePower;

            // Optionally, add a forward-only approach if needed based on additional criteria
            // For example, based on the size of the bounding box indicating distance

            // Stop motors if alignment is within acceptable threshold
            if (Math.abs(errorX) < 10) { // Allowable error for alignment
                frontLeftPower = 0;
                frontRightPower = 0;
                backLeftPower = 0;
                backRightPower = 0;
                telemetry.addData("Alignment", "Aligned!");
            }

        } else {
            // Standard drive mode
            movementSpeedMultiplier = gamepad1.left_bumper ? 0.5 : 1;

            double drive = -gamepad1.left_stick_y * movementSpeedMultiplier;
            double strafe = gamepad1.left_stick_x * 0.5 * movementSpeedMultiplier; // Reduce strafing speed
            double turn = gamepad1.right_stick_x * movementSpeedMultiplier;

            frontLeftPower = drive + strafe + turn;
            frontRightPower = drive - strafe - turn;
            backLeftPower = drive - strafe + turn;
            backRightPower = drive + strafe - turn;

            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);
        }


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

        armBasePower *= 0.05;

        // Send calculated power to arm motors
        core.armBaseMotor.setPower(armBasePower);
        core.jointOneMotor.setPower(jointOnePower);
        // Send calculated power to arm motor
        core.armBaseMotor.setPower(armBasePower);

        // Show the elapsed game time and wheel power.
        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Arm Base Power", armBasePower);
        telemetry.addData("Joint One Power", jointOnePower);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Alignment Mode", alignMode ? "ON" : "OFF");
        telemetry.addData("Target Color", targetBlue ? "Blue" : "Red");

        //telemetry.addData("Center X", centerX);
        //telemetry.addData("Error X", centerX - 160);
        telemetry.update();
    }

    static class GamePiecePipeline extends OpenCvPipeline {
        static volatile double centerX = 0;

        @Override
        public Mat processFrame(Mat input) {
            Scalar lowerBound, upperBound;

            // Define color ranges for detection
            if (targetBlue) {
                lowerBound = new Scalar(100, 100, 100); // Blue range, adjust values
                upperBound = new Scalar(130, 255, 255);
            } else {
                lowerBound = new Scalar(0, 100, 100);   // Red range, adjust values
                upperBound = new Scalar(10, 255, 255);
            }

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Mat mask = new Mat();
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // Get bounding rectangle
            Rect boundingRect = Imgproc.boundingRect(mask);

            // Check if any object is detected
            if (boundingRect.width > 0 && boundingRect.height > 0) {
                Point topLeft = boundingRect.tl();
                Point bottomRight = boundingRect.br();

                // Calculate the center X of the bounding box
                centerX = (topLeft.x + bottomRight.x) / 2;

                // Draw rectangle and center point for visualization
                Imgproc.rectangle(input, topLeft, bottomRight, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(centerX, (topLeft.y + bottomRight.y) / 2), 5, new Scalar(255, 0, 0), -1);
            } else {
                // No object detected
                centerX = 160; // Assume centered if nothing is detected
            }

            mask.release();
            hsv.release();

            return input;
        }
    }
}