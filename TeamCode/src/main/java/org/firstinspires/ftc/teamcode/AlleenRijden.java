package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//@Disabled
@TeleOp(name="AlleenRijden", group="AAALinear Opmode")
public class AlleenRijden extends LinearOpMode {

    public static double maxspeed = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftback;
    private DcMotor rightback;
    private DcMotor leftfront;
    private DcMotor rightfront;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() {

        leftfront = hardwareMap.dcMotor.get(ConfigurationName.leftfront);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setDirection(DcMotor.Direction.FORWARD);

        leftback = hardwareMap.dcMotor.get(ConfigurationName.leftback);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setDirection(DcMotor.Direction.FORWARD);

        rightfront = hardwareMap.dcMotor.get(ConfigurationName.rightfront);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setDirection(DcMotor.Direction.REVERSE);

        rightback = hardwareMap.dcMotor.get(ConfigurationName.rightback);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double head = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(head) - y * Math.sin(head);
            double rotY = x * Math.sin(head) + y * Math.cos(head);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * maxspeed;
            double backLeftPower = ((rotY - rotX + rx) / denominator) * maxspeed;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * maxspeed;
            double backRightPower = ((rotY + rotX - rx) / denominator) * maxspeed;

            leftfront.setPower(frontLeftPower);
            leftback.setPower(backLeftPower);
            rightfront.setPower(frontRightPower);
            rightback.setPower(backRightPower);

            telemetry.addData("Motors", "leftfront (%.2f)", frontLeftPower);
            telemetry.addData("Motors", "leftback (%.2f)", backLeftPower);
            telemetry.addData("Motors", "rightfront (%.2f)", frontRightPower);
            telemetry.addData("Motors", "rightback (%.2f)", backRightPower);
            telemetry.addData("IMU", "Radians (%.2f)", head);
        }
    }
}