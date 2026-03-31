package frc.robot;


    import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
    
    import edu.wpi.first.math.geometry.Rotation3d;
    import edu.wpi.first.math.geometry.Transform3d;
    import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Preferences;

public final class Constants {

  public static final class IntakeConstants {

        // Roller Motor ID (CAN ID from Phoenix Tuner)
        public static final int ROLLER_MOTOR_ID = 14;

        // Roller settings
        public static final boolean kRollerInverted = false;

        public static final double kIntakeSpeed = -0.4; //VELOCIDAD DE INTAKE
        public static final double kOuttakeSpeed = 0.4; //VELOCIDAD DE OUTTAKE
  }

  



  public static final class IntakePivotConstants {

        // Roller Motor ID (CAN ID from REV Hardware Client)
        public static final int PIVOT_RIGHT_MOTOR_ID =19;

        // Pivot settings
        public static final boolean kPivotRightInverted = true;
        public static final IdleMode kPivotRightIdleMode =IdleMode.kCoast;

        //Current limit (amps)
        public static final int kPivotCurrentLimit = 20;

        public static final double kPivotUpSpeed = 0.3;
        public static final double kPivotDownSpeed = -0.3;


  }

  /// PHOTON VISION CAMS
/// OrangePiRightCam = registerd right cams 10.63.48.12
/// RIGHT_FRONT_CAM
/// RIGHT_CAM
/// OrangePiLeftCam = registered Left cams 10.63.48.11
/// LEFT_FRONT_CAM
/// LEFT_CAM
/// 
    
     public final class ShooterConstants {

         // CAN IDs SEGUN PHOENIX TUNER
         public static final int shooterLEFT_ID = 16;
         public static final int shooterRIGHT_ID = 17;
         public static final int shooterINDEXER_ID = 22;
         public static final int shooterHOPPER_ID = 27;

         // Inversion (true if the motor spins opposite of desired)
         public static final boolean shooterLEFT_INVERTED = true;
         public static final boolean shooterRIGHT_INVERTED = false;
         public static final boolean shooterINDEXER_INVERTED = true;
         public static final boolean shooterHOPPER_INVERTED = true;


         // Default speeds (percent output or RPM)
         public static final double kShooterSpeed = 0.46;   // PERCENTAGE OUTPUT FOR SHOOTER
         public static final double kIndexerSpeed = 0.6;  // Example percent output for indexer
         public static final double kHopperSpeed = 0.6;  // PERCENTAGE OUTPUT FOR HOPPER

         public static final double shooterMiddleDutyCycle = 0.335; // PERCENTAGE OUTPUT PARA MEDIA CANCHA
         public static final double shooterCloseDutyCycle = 0.44; // PERCENTAGE OUTPUT PARA CERCA DE LA PORTERIA
         public static final double shooterLongDutyCycle = 0.7; // PERCENTAGE OUTPUT PARA LARGA DISTANCIA
        public static final double COMPENSATION_MULTIPLICATOR = 1.1;

     private ShooterConstants() {
        // Prevent instantiation PONER SIEMPRE QUE USES KRAKENS
    }

    
}

    // =========================================================================
    // Vision (PhotonVision — 4 camaras, 2 coprocesadores)
    // =========================================================================

    /**
     * Constantes del sistema de vision con PhotonVision 2026.
     *
     * <h2>Arquitectura</h2>
     * <ul>
     *   <li>2 coprocesadores, 2 camaras cada uno.</li>
     *   <li>Cada camara corre en hilo daemon independiente.</li>
     *   <li>Los nombres de camara deben coincidir con los de la UI de PhotonVision.</li>
     *   <li>Los Transform3d deben medirse con cinta en el robot fisico.</li>
     * </ul>
     */
    public static final class VisionConstants {

        // --- IPs de los coprocesadores (agrupados por lado fisico del robot) ---
        public static final String COPROCESSOR_1_IP = "10.63.48.11"; // Lado DERECHO
        public static final String COPROCESSOR_2_IP = "10.63.48.12"; // Lado IZQUIERDO

        // --- Nombres de las camaras (exactos, incluyendo mayusculas) ---
        // Verificar que coincidan con los nombres en la UI de PhotonVision:
        //   http://10.63.48.11:5800  (derecho)  → CAM_A1_NAME, CAM_B1_NAME
        //   http://10.63.48.12:5800  (izquierdo) → CAM_A2_NAME, CAM_B2_NAME
        public static final String CAM_A1_NAME = "FRONT_RIGHT_CAM"; // Coproc 1 (derecho)
        public static final String CAM_A2_NAME = "FRONT_LEFT_CAM";  // Coproc 2 (izquierdo)
        public static final String CAM_B1_NAME = "RIGHT_CAM";       // Coproc 1 (derecho)
        public static final String CAM_B2_NAME = "LEFT_CAM";        // Coproc 2 (izquierdo)
        
        // --- Transforms robot → camara ---
        // Transform3d(Translation3d(adelante_m, izq_m, arriba_m), Rotation3d(roll, pitch, yaw))
        public static final Transform3d ROBOT_TO_CAM_A1 = new Transform3d(
            new Translation3d(0.2794,  0.2794, 0.34),
            new Rotation3d(0.0, 0, 0.0)
        );
        public static final Transform3d ROBOT_TO_CAM_A2 = new Transform3d(
            new Translation3d(-0.2794, 0.2794, 0.34),
            new Rotation3d(0.0, 0, 0.0)
        );
        public static final Transform3d ROBOT_TO_CAM_B1 = new Transform3d(
            new Translation3d(0.3429,  0.26035, 0.353),
            new Rotation3d(0.0, 0, Math.toRadians(180.0))
        );
        public static final Transform3d ROBOT_TO_CAM_B2 = new Transform3d(
            new Translation3d(-0.3429, 0.26035, 0.353),
            new Rotation3d(0.0, 0, Math.toRadians(180.0))
        );

        // =========================================================================
        // Std devs fijos por camara — ajustar manualmente segun calibracion
        // =========================================================================
        // SEMANTICA: [x_metros, y_metros, theta_radianes]
        //   MAS ALTO = MENOS confianza  (el filtro corrige menos la odometria)
        //   MAS BAJO = MAS confianza    (el filtro corrige mas la odometria)
        //
        // Para desactivar una camara completamente: pon STD_XY en 9999.0
        //
        // POR QUE STD_THETA = 9999 EN TODAS:
        // El heading lo maneja el giroscopio (Pigeon2), que es ~100x mas preciso
        // que vision. Si vision modifica theta (4 camaras x 50 Hz), pelea con el
        // gyro y el "frente" del field-centric cambia constantemente. 9999 = el
        // filtro de Kalman ignora el theta de vision por completo.
        // El unico knob util para calibrar es STD_XY, uno por camara.

        // FRONT_RIGHT_CAM
        public static final double CAM_A1_STD_XY    = 0.8;
        public static final double CAM_A1_STD_THETA = 9999.0;

        // RIGHT_CAM
        public static final double CAM_B1_STD_XY    = 0.8;
        public static final double CAM_B1_STD_THETA = 9999.0;

        // FRONT_LEFT_CAM
        public static final double CAM_A2_STD_XY    = 0.8;
        public static final double CAM_A2_STD_THETA = 9999.0;

        // LEFT_CAM
        public static final double CAM_B2_STD_XY    = 0.8;
        public static final double CAM_B2_STD_THETA = 9999.0;

        // =========================================================================
        // Filtros de calidad y rechazo de outliers
        // =========================================================================
        /** Rechazar estimaciones con ambiguedad mayor a este valor [0.0–1.0]. */
        public static final double MAX_POSE_AMBIGUITY = 0.2;

        /** Rechazar tags a mas de esta distancia [m]. */
        public static final double MAX_TAG_DISTANCE_METERS = 6.0;

        /** Si la pose de vision difiere mas de esto de la odometria actual, se rechaza.
         *  Protege contra glitches de deteccion de tags que jalarian al robot de golpe. [m] */
        public static final double MAX_VISION_POSE_JUMP_METERS = 50.0;

        /**
         * Velocidad maxima del robot para aceptar correcciones de vision [m/s].
         *
         * Por que existe esto:
         *   Cuando el robot se mueve rapido, el filtro de Kalman hace "replay" de odometria
         *   cada vez que vision manda una correccion (4 camaras x 50Hz = ~200/s). Odometria
         *   y vision jalan en direcciones opuestas y la oscilacion se acumula como distancia
         *   extra. A alta velocidad ademas la vision es menos precisa (latencia, motion blur).
         *
         *   Con 1.5 m/s: vision corrige cuando el robot esta casi parado o en movimiento lento
         *   (por ejemplo al apuntar para disparar). En movimiento rapido la odometria manda.
         *
         *   Subir si el robot corrige pose durante el auto; bajar si siguen los saltos.
         */
        public static final double VISION_MAX_SPEED_MPS = 1.5;
    }

     // =========================================================================
    // Field (Geometria del campo FRC 2026)
    // =========================================================================

    /**
     * Geometria del campo FRC 2026 — dos variantes: Welded y Andymark.
     *
     * <h2>Como cambiar el tipo de campo</h2>
     * <ol>
     *   <li>Abrir SmartDashboard o Shuffleboard.</li>
     *   <li>Ir a la tabla <b>Preferences</b>.</li>
     *   <li>Cambiar <b>IsAndymarkField</b> a {@code true} (Andymark) o {@code false} (Welded).</li>
     *   <li><b>Reiniciar el codigo del robot</b> para que el cambio tome efecto.</li>
     * </ol>
     *
     * Origen del campo: esquina Red (x=0, y=0) segun convencion WPILib.
     * Eje X: de alianza Red hacia alianza Blue (longitud del campo).
     * Eje Y: de la pared sur hacia la pared norte.
     *
     * <h2>Zonas del campo</h2>
     * <ul>
     *   <li><b>RED_ALLIANCE</b>:  x &lt; RED_ZONE_MAX_X</li>
     *   <li><b>NEUTRAL</b>:       RED_ZONE_MAX_X &le; x &le; BLUE_ZONE_MIN_X</li>
     *   <li><b>BLUE_ALLIANCE</b>: x &gt; BLUE_ZONE_MIN_X</li>
     * </ul>
     */
    public static final class FieldConstants {

        public enum FieldType { WELDED, ANDYMARK }

        /**
         * Tipo de campo activo — determinado desde Preferences al arrancar.
         * Preferences key: {@code IsAndymarkField} (boolean).
         * Default: {@code false} → Welded.
         *
         * <p>Requiere reinicio del robot para aplicar cambios.</p>
         */
        public static final FieldType ACTIVE_FIELD_TYPE;

        static {
            // initBoolean crea la clave con el valor por defecto si no existe.
            // Si ya existe (guardada en flash del RoboRIO), no la sobrescribe.
            Preferences.initBoolean("IsAndymarkField", false);
            ACTIVE_FIELD_TYPE = Preferences.getBoolean("IsAndymarkField", false)
                ? FieldType.ANDYMARK : FieldType.WELDED;
        }

        // =====================================================================
        // Dimensiones brutas — convertidas de pulgadas a metros (1" = 0.0254 m)
        // =====================================================================

        // --- Welded field: 651.22" x 317.69" ---
        public static final double W_LEN =  651.22 * 0.0254; // 16.5410 m
        public static final double W_WID =  317.69 * 0.0254; //  8.0693 m
        // Scoring target X desde cada borde [182.11"]
        public static final double W_SCR =  182.11 * 0.0254; //  4.6256 m
        // Passing target X desde cada borde [91.00"]
        public static final double W_PSX =   91.00 * 0.0254; //  2.3114 m
        // Passing target Y desde cada borde [79.42"]
        public static final double W_PSY =   79.42 * 0.0254; //  2.0173 m

        // --- Andymark field: 650.12" x 316.64" ---
        public static final double A_LEN =  650.12 * 0.0254; // 16.5130 m
        public static final double A_WID =  316.64 * 0.0254; //  8.0427 m
        // Scoring target X desde cada borde [181.56"]
        public static final double A_SCR =  181.56 * 0.0254; //  4.6116 m
        // Passing target X desde cada borde [91.00"] — igual que Welded
        public static final double A_PSX =   91.00 * 0.0254; //  2.3114 m
        // Passing target Y desde cada borde [79.42"] — igual que Welded
        public static final double A_PSY =   79.42 * 0.0254; //  2.0173 m

        // =====================================================================
        // Dimensiones activas (seleccionadas por ACTIVE_FIELD_TYPE)
        // =====================================================================
        // IMPORTANTE: estas constantes deben declararse DESPUES del bloque static{}
        // para que ACTIVE_FIELD_TYPE ya este inicializado cuando sel() se llama.

        /** Longitud total del campo (eje X) [m]. */
        public static final double FIELD_LENGTH = sel(W_LEN, A_LEN);

        /** Ancho total del campo (eje Y) [m]. */
        public static final double FIELD_WIDTH  = sel(W_WID, A_WID);

        // --- Limites de zona ---

        /** X maximo de la zona Red Alliance [m]. Todo x < este valor es zona roja. */
        public static final double RED_ZONE_MAX_X  = sel(W_SCR, A_SCR);

        /** X minimo de la zona Blue Alliance [m]. Todo x > este valor es zona azul. */
        public static final double BLUE_ZONE_MIN_X = FIELD_LENGTH - RED_ZONE_MAX_X;

        // --- Scoring targets (simetricos, centrados en Y) ---

        public static final double RED_SCORING_X  = sel(W_SCR, A_SCR);
        public static final double RED_SCORING_Y  = FIELD_WIDTH / 2.0;
        public static final double BLUE_SCORING_X = FIELD_LENGTH - RED_SCORING_X;
        public static final double BLUE_SCORING_Y = FIELD_WIDTH  / 2.0;

        // --- Passing targets ---
        // PASS_1 = lado sur (Y bajo); PASS_2 = lado norte (Y alto)

        public static final double RED_PASS_X    = sel(W_PSX, A_PSX);
        public static final double BLUE_PASS_X   = FIELD_LENGTH - RED_PASS_X;
        public static final double PASS_Y_SOUTH  = sel(W_PSY, A_PSY);
        public static final double PASS_Y_NORTH  = FIELD_WIDTH - PASS_Y_SOUTH;

        // =====================================================================
        // Selector interno
        // =====================================================================

        public static double sel(double welded, double andymark) {
            return (ACTIVE_FIELD_TYPE == FieldType.WELDED) ? welded : andymark;
        }
    }


}

    




