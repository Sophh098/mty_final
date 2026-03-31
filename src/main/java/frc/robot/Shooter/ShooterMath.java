package frc.robot.Shooter;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Clase utilitaria para calcular la distancia entre el robot y el objetivo
 * de anotacion, e interpolar las RPM del flywheel necesarias para ese disparo.
 *
 * <h2>Uso tipico</h2>
 * <pre>{@code
 * ShooterMath.ShooterResult r = ShooterMath.calculate(
 *     drivetrain.getState().Pose,
 *     DriverStation.getAlliance().orElse(Alliance.Blue)
 * );
 * ShooterMath.publishToSmartDashboard(r);
 * if (r.isValidShot()) {
 *     flywheel.setRPM(r.flywheelRPM());
 * }
 * }</pre>
 */
public final class ShooterMath {

    private ShooterMath() {
        throw new UnsupportedOperationException("ShooterMath es una clase utilitaria, no debe instanciarse.");
    }

    // =========================================================================
    // Lookup table: distancia (m) → RPM del flywheel. Hood fijo en todo momento.
    // =========================================================================

    private static final TreeMap<Double, Double> FIXED_HOOD_RPM_TABLE = new TreeMap<>();
    static {
        FIXED_HOOD_RPM_TABLE.put(0.6, 112.787  * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(0.8, 150.383  * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.0, 187.978  * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.2, 1039.278 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.4, 1061.717 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.6, 1092.640 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(1.8, 1127.238 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.0, 1163.383 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.2, 1200.032 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.4, 1236.641 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.6, 1272.917 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(2.8, 1308.702 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.0, 1343.913 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.2, 1378.510 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.4, 1412.480 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.6, 1445.825 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(3.8, 1478.556 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.0, 1510.690 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.2, 1542.245 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.4, 1573.244 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.6, 1603.707 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(4.8, 1633.655 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.0, 1663.109 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.2, 1692.089 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.4, 1720.615 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.6, 1748.704 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(5.8, 1776.375 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.0, 1803.643 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.2, 1830.525 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.4, 1857.035 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.6, 1883.187 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(6.8, 1908.994 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.0, 1934.470 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.2, 1959.626 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.4, 1984.472 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.6, 2009.020 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(7.8, 2033.280 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
        FIXED_HOOD_RPM_TABLE.put(8.0, 2057.262 * ShooterConstants.COMPENSATION_MULTIPLICATOR);
    }

    // =========================================================================
    // Resultado del calculo
    // =========================================================================

    /**
     * Resultado del calculo de disparo.
     *
     * @param flywheelRPM    RPM requeridas del flywheel interpoladas de la tabla.
     * @param distanceMeters Distancia horizontal robot-objetivo [m].
     * @param isValidShot    {@code true} si la distancia esta dentro del rango valido.
     * @param invalidReason  Descripcion del fallo (cadena vacia si valido).
     */
    public record ShooterResult(
        double  flywheelRPM,
        double  distanceMeters,
        boolean isValidShot,
        String  invalidReason
    ) {
        /** Resultado invalido con valores numericos a 0 y una razon. */
        public static ShooterResult invalid(String reason) {
            return new ShooterResult(0.0, 0.0, false, reason);
        }
    }

    // =========================================================================
    // Calculo principal
    // =========================================================================

    /**
     * Calcula la distancia al scoring target y las RPM necesarias para ese disparo.
     *
     * @param robotPose Pose actual del robot (de la odometria).
     * @param alliance  Alianza actual del robot.
     * @return {@link ShooterResult} con distancia y RPM interpoladas.
     */
    public static ShooterResult calculate(Pose2d robotPose, Alliance alliance) {
        Translation2d targetXY = (alliance == Alliance.Blue)
            ? new Translation2d(FieldConstants.BLUE_SCORING_X, FieldConstants.BLUE_SCORING_Y)
            : new Translation2d(FieldConstants.RED_SCORING_X,  FieldConstants.RED_SCORING_Y);

        double distance = robotPose.getTranslation().getDistance(targetXY);

        if (distance <= 0.0) {
            return ShooterResult.invalid(String.format(
                "Distancia d=%.2f m invalida (demasiado cerca del objetivo).", distance));
        }

        double rpm = interpolateTable(FIXED_HOOD_RPM_TABLE, distance);

        return new ShooterResult(rpm, distance, true, "");
    }

    // =========================================================================
    // SmartDashboard
    // =========================================================================

    /**
     * Publica el resultado en SmartDashboard bajo el prefijo {@code "Shooter/"}.
     *
     * @param result Resultado a publicar.
     */
    public static void publishToSmartDashboard(ShooterResult result) {
        SmartDashboard.putNumber ("Shooter/FlywheelRPM",   result.flywheelRPM());
        SmartDashboard.putNumber ("Shooter/Distance",      result.distanceMeters());
        SmartDashboard.putBoolean("Shooter/ValidShot",     result.isValidShot());
        SmartDashboard.putString ("Shooter/InvalidReason", result.invalidReason());
    }

    // =========================================================================
    // Utilitario interno
    // =========================================================================

    /**
     * Interpolacion lineal sobre una lookup table ordenada por clave.
     * Si {@code x} esta fuera del rango, retorna el valor del extremo mas cercano (clamp).
     */
    private static double interpolateTable(TreeMap<Double, Double> table, double x) {
        Map.Entry<Double, Double> lo = table.floorEntry(x);
        Map.Entry<Double, Double> hi = table.ceilingEntry(x);
        if (lo == null) return hi.getValue();
        if (hi == null) return lo.getValue();
        if (lo.getKey().equals(hi.getKey())) return lo.getValue();
        double t = (x - lo.getKey()) / (hi.getKey() - lo.getKey());
        return lo.getValue() + t * (hi.getValue() - lo.getValue());
    }
}