package org.firstinspires.ftc.teamcode.juniper.wrappers;

import androidx.annotation.NonNull;

import com.example.juniper.util.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class nServo extends Hardware {

    public enum ServoType {
        AXON_MINI_V1 (Math.toRadians(60) / 0.11, 355),
        AXON_MAX_V1 (Math.toRadians(60) / 0.16, 355),
        AXON_MINI_V2 (Math.toRadians(60) / 0.11, 355),
        AXON_MAX_V2 (Math.toRadians(60) / 0.16, 355),
        AXON_MINI_SPM(Math.toRadians(60) / 0.1, 355),
        AXON_MAX_SPM(Math.toRadians(60) / 0.12, 355),
        GB_TORQUE(Math.toRadians(60) / 0.25, 300),
        GB_TORQUE_SPM(Math.toRadians(60) / 0.2, 300),
        GB_SPEED(Math.toRadians(60) / 0.11, 300),
        GB_SPEED_SPM(Math.toRadians(60) / 0.09, 300),
        GB_SUPER_SPEED(Math.toRadians(60) / 0.055, 300),
        GB_SUPER_SPEED_SPM(Math.toRadians(60) / 0.043, 300);

        private final double speed, rangeInRadian;

        ServoType(double speed, double rangeInDegrees) {
            this.speed = speed;
            this.rangeInRadian = Math.toRadians(rangeInDegrees);
        }

        public double getSpeed() {
            return speed;
        }

        public double getRangeInRadians() {
            return rangeInRadian;
        }
    }

    private final double speed, rangeInRadians; // speed in units of radians / sec
    private final Servo[] servos;
    private final boolean[] reversed;
    private long lastLoop;
    private double currentAngle = 0, targetAngle = 0, power = 1.0;

    private double max = 1, min = 0;

    public nServo (String[] names, double basePrio, double prioScale, boolean[] reversed, String properName, HardwareMap map, ServoType type) {
        this(names, basePrio, prioScale, reversed, properName, map, type.getSpeed(), type.getRangeInRadians());
    }

    public nServo (String[] names, boolean[] reversed, String properName, HardwareMap map, ServoType type) {
        this(names, 0, 1, reversed, properName, map, type.getSpeed(), type.getRangeInRadians());
    }

    public nServo (String[] names, double basePrio, double prioScale, boolean[] reversed, String properName, HardwareMap map, double speed, double rangeInRadians) {
        super(properName, basePrio, prioScale, map);

        servos = new Servo[names.length];
        for (int i = 0; i < names.length; i++) {
            servos[i] = map.get(Servo.class, names[i]);
        }

        this.reversed = reversed;

        this.speed = speed;
        this.rangeInRadians = rangeInRadians;

        lastLoop = System.nanoTime();
        currentAngle = convertPosToRad(servos[0].getPosition());
        targetAngle = currentAngle;
    }

    public nServo (String[] names, double basePrio, double prioScale, boolean[] reversed, String properName, HardwareMap map, double speed, double rangeInRadians, double min, double max) {
        this(names, basePrio, prioScale, reversed, properName, map, speed, rangeInRadians);
        this.max = max;
        this.min = min;
    }

    public double convertPosToRad(double pos) {
        return pos * rangeInRadians;
    }
    public double convertPosToDegrees(double pos) {
        return Math.toDegrees(pos * rangeInRadians);
    }

    public double convertRadToPos(double ang) {
        return ang / rangeInRadians;
    }
    public double convertDegreesToPos(double ang) {
        return Math.toRadians(ang) / rangeInRadians;
    }

    @Override
    public void update() {
        long currentTime = System.nanoTime();
        double timeSinceLast = ((double) currentTime - lastLoop) / 1.0E9;

        double error = targetAngle - currentAngle;
        double deltaAngle = timeSinceLast * speed * power * Math.signum(error);

        currentAngle += deltaAngle;

        //straight to if change is too small or power is one
        if (Math.abs(deltaAngle) > Math.abs(error) || power == 1) {
            currentAngle = targetAngle;
        }

        if (Math.abs(targetAngle - currentAngle) > Math.toRadians(0.8)) {
            for (int i = 0; i < servos.length; i++) {
                double pos = convertRadToPos(currentAngle);
                if (reversed[i]) {
                    pos = 1 - pos;
                }
                servos[i].setPosition(pos);
            }
        }

        lastLoop = currentTime;
    }

    @Override
    public double calculatePriority() {
        double delta = Math.abs(targetAngle - currentAngle);
        if (delta == 0) {
            currentPrio = 0;
            return currentPrio;
        }

        double prio = basePriority;

        long currentTime = System.nanoTime();
        double timeSinceLast = ((double) currentTime - lastLoop) / 1.0E9;
        prio += (timeSinceLast * 0.5);

        prio += (delta * 10);

        if (forceUpdate) {
            prio += 100000;
            forceUpdate = false;
        }

        this.currentPrio = prio * priorityScale;
        return currentPrio;
    }

    public void forceServoPos (double pos) {
        for (int i = 0; i < servos.length; i++) {
            double temp = pos;
            if (reversed[i]) {
                temp = 1 - pos;
            }
            servos[i].setPosition(temp);
        }
    }

    public void setMaxServoPos (double pos) {
        this.max = Utils.minMaxClip(pos, min, 1);
    }
    public void setMinServoPos (double pos) {
        this.min = Utils.minMaxClip(pos, 0, max);
    }

    public void setTargetAngle (double angle) {
        setTargetAngle(angle, 0.99);
    }
    public void setTargetAngle (double angle, double power) {
        targetAngle = Utils.minMaxClip(angle, convertPosToRad(min), convertPosToRad(max)); // limits to 0 and 1, prolly need to setup base pos soon
        this.power = Utils.minMaxClip(power, 0.000001, 1);
    }

    public void setTargetPos(double pos) {
        setTargetPos(pos, 0.99);
    }
    public void setTargetPos(double pos, double power) {
        setTargetAngle(convertPosToRad(pos), power);
    }

    public boolean inPosition() {
        return Math.abs(targetAngle - currentAngle) < Math.toRadians(0.8);
    }
    public boolean inPosition(double tol) {
        return Math.abs(targetAngle - currentAngle) < tol;
    }

    public double getTargetAngle () {
        return targetAngle;
    }
    public double getTargetPos() {
        return convertRadToPos(targetAngle);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }
    public double getCurrentPosition() {
        return convertRadToPos(currentAngle);
    }

    public double getMaxRadians() {
        return rangeInRadians;
    }

    @NonNull
    @Override
    public String toString() {
        return "Servo " + name + ": \n" +
                "current angle " + currentAngle + "\n" +
                "current position " + convertRadToPos(currentAngle) + "\n" +
                "target angle " + targetAngle  + "\n" +
                "target position " + convertRadToPos(targetAngle) + "\n" +
                "in position " + inPosition() + "\n" +
                "power " + power + "\n"
                ;
    }
}
