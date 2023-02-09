package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Jake_2_Hardware {
    // Motors and Servos
    public DcMotor MotorVL;//port0
    public DcMotor MotorVR;//port2
    public DcMotor MotorHL;//port1
    public DcMotor MotorHR;//port3
    public DcMotorEx MotorLift;
    public CRServo IntakeS;
    public Servo AlignmentBar;
    public DistanceSensor BaseDS;
    public DistanceSensor IntakeDS;
    public ColorRangeSensor IntakeLeftColor;
    public ColorRangeSensor IntakeRightColor;


    BNO055IMU imu;

    Orientation angles;






    HardwareMap testhardware;

    public void init(HardwareMap testhardware){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();



        imu = testhardware.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        //colorSenor = testhardware.get(ColorRangeSensor.class, "ColorSensor");



        // Define motors and servos
        IntakeLeftColor = testhardware.get(ColorRangeSensor.class, "IntakeLeftColor");
        IntakeRightColor = testhardware.get(ColorRangeSensor.class, "IntakeRightColor");
        BaseDS = testhardware.get(DistanceSensor.class, "BaseDS");
        IntakeS = testhardware.get(CRServo.class, "IntakeS");
        MotorVL = testhardware.get(DcMotorEx.class, "MotorVL");
        MotorVR = testhardware.get(DcMotorEx.class, "MotorVR");
        MotorHL = testhardware.get(DcMotorEx.class, "MotorHL");
        MotorHR = testhardware.get(DcMotorEx.class, "MotorHR");
        MotorLift = testhardware.get(DcMotorEx.class, "MotorLift");
        AlignmentBar = testhardware.get(Servo.class, "AlignmentBar");
        IntakeDS = testhardware.get(DistanceSensor.class, "IntakeDS");


        MotorVL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorVR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorHL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorHR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorVL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorVR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorHL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorHR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*MotorVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorVR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorHL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorHR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        MotorVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorVR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorHL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorHR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        /*Dist1 = testhardware.get(DistanceSensor.class, "Dist1");
        Dist2 = testhardware.get(DistanceSensor.class, "Dist2");
        Dist3 = testhardware.get(DistanceSensor.class, "Dist3");
        Dist4 = testhardware.get(DistanceSensor.class, "Dist4");
        Dist5 = testhardware.get(DistanceSensor.class, "Dist5");
        Dist6 = testhardware.get(DistanceSensor.class, "Dist6");
        Dist7 = testhardware.get(DistanceSensor.class, "Dist7");
        Dist8 = testhardware.get(DistanceSensor.class, "Dist8");
*/
    }
}
