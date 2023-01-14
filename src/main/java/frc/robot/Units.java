package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DrivetrainConstants;

public class Units {
    private static final double kInchesPerFoot = 12.0;
    private static final double kMetersPerInch = 0.0254;
    private static final double kSecondsPerMinute = 60;

    public static class DrivetrainUnits {
        private static final double kTicksPerRotation = DrivetrainConstants.kTicksPerRotation;
        private static final double kWheelDiameter = DrivetrainConstants.kWheelDiameter;
        private static final double kDecisecondsPerSecond = 10;

        /**
         * Converts given meters/sec to ticks/ds
         * 
         * @param metersPerSecond The meters/sec to convert to ticks/ds
         * @return Ticks/ds converted from meters/sec
         */
        public static double MPSToTicksPerDecisecond(double metersPerSecond) {
            return MetersToTicks(metersPerSecond / kDecisecondsPerSecond);
        }

        /**
         * Converts given feet/sec to ticks/ds
         * 
         * @param feetPerSecond The feet/sec to convert to ticks/ds
         * @return Ticks/ds converted from feet/sec
         */
        public static double FPSToTicksPerDecisecond(double feetPerSecond) {
            return MPSToTicksPerDecisecond(FeetToMeters(feetPerSecond));
        }

        /**
         * Converts given ticks/ds to meters/sec
         * 
         * @param ticksPerDecisecond The ticks/ds to convert to meters/s
         * @return Meters/s converted from ticks/ds
         */
        public static double TicksPerDecisecondToMPS(double ticksPerDecisecond) {
            return TicksToMeters(kDecisecondsPerSecond * ticksPerDecisecond);
        }

        /**
         * Converts given ticks/ds to feet/sec
         * 
         * @param ticksPerDecisecond The ticks/ds to convert to feet/sec
         * @return Feet/sec converted from ticks/ds
         */
        public static double TicksPerDecisecondToFPS(double ticksPerDecisecond) {
            return MetersToFeet(TicksToMeters(kDecisecondsPerSecond * ticksPerDecisecond));
        }

        /**
         * Converts given ticks to meters
         * 
         * @param ticks The ticks to convert to meters
         * @return Meters converted from ticks
         */
        public static double TicksToMeters(double ticks) {
            return ticks / kTicksPerRotation * Math.PI * kWheelDiameter;
        }

        /**
         * Converts given meters to ticks
         * 
         * @param meters The meters to convert to ticks
         * @return Ticks converted from meters
         */
        public static double MetersToTicks(double meters) {
            return meters * kTicksPerRotation / Math.PI / kWheelDiameter;
        }

        /**
         * Converts a Pose2d in meters to a Pose2d in feet
         * 
         * @param meter The Pose2d to convert to feet
         * @return Pose2d in feet converted from meters
         */
        public static Pose2d MeterPoseToFeetPose(Pose2d meter) {
            Translation2d feetTranslation = meter.getTranslation().div(kMetersPerInch * kInchesPerFoot);
            return new Pose2d(feetTranslation, meter.getRotation());
        }

    }

    /**
     * Converts given meters to feet.
     *
     * @param meters The meters to convert to feet.
     * @return Feet converted from meters.
     */
    public static double MetersToFeet(double meters) {
        return MetersToInches(meters) / kInchesPerFoot;
    }

    /**
     * Converts given feet to meters.
     *
     * @param feet The feet to convert to meters.
     * @return Meters converted from feet.
     */
    public static double FeetToMeters(double feet) {
        return InchesToMeters(feet * kInchesPerFoot);
    }

    /**
     * Converts given meters to inches.
     *
     * @param meters The meters to convert to inches.
     * @return Inches converted from meters.
     */
    public static double MetersToInches(double meters) {
        return meters / kMetersPerInch;
    }

    /**
     * Converts given inches to meters.
     *
     * @param inches The inches to convert to meters.
     * @return Meters converted from inches.
     */
    public static double InchesToMeters(double inches) {
        return inches * kMetersPerInch;
    }

    /**
     * Converts rotations per minute to radians per second.
     *
     * @param rpm The rotations per minute to convert to radians per second.
     * @return Radians per second converted from rotations per minute.
     */
    public static double RotationsPerMinuteToRadiansPerSecond(double rpm) {
        return rpm * Math.PI / (kSecondsPerMinute / 2);
    }

    /**
     * Converts radians per second to rotations per minute.
     *
     * @param radiansPerSecond The radians per second to convert to from rotations
     *                         per minute.
     * @return Rotations per minute converted from radians per second.
     */
    public static double RadiansPerSecondToRotationsPerMinute(double radiansPerSecond) {
        return radiansPerSecond * (kSecondsPerMinute / 2) / Math.PI;
    }
}