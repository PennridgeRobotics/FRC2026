package frc.robot.util.dashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.List;
import java.util.Objects;
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class PIDSendable implements Sendable {
    private final int type;
    private final @Nullable SparkMax sparkMax;
    private final @Nullable ClosedLoopSlot closedLoopSlot;
    private final @Nullable PIDController pidController;
    private final @Nullable ProfiledPIDController profiledPIDController;
    private final @Nullable ElevatorFeedforward elevatorFeedforward;
    private final @Nullable ArmFeedforward armFeedforward;
    private final PIDValues sparkMaxPIDValues;

    private PIDSendable(
            int type,
            @Nullable PIDValues defaults,
            @Nullable SparkMax sparkMax,
            @Nullable ClosedLoopSlot closedLoopSlot,
            @Nullable PIDController pidController,
            @Nullable ProfiledPIDController profiledPIDController,
            @Nullable ElevatorFeedforward elevatorFeedforward,
            @Nullable ArmFeedforward armFeedforward) {
        this.type = type;
        this.sparkMaxPIDValues = defaults != null ? defaults.copy() : new PIDValues();
        this.sparkMax = sparkMax;
        this.closedLoopSlot = closedLoopSlot;
        this.pidController = pidController;
        this.profiledPIDController = profiledPIDController;
        this.elevatorFeedforward = elevatorFeedforward;
        this.armFeedforward = armFeedforward;
        if ((type & Type.G) != 0 && (type & Type.COS) != 0) {
            throw new IllegalArgumentException("Cannot have both G and COS feedforward types");
        }
    }

    // Spark MAX
    public PIDSendable(SparkMax sparkMax, ClosedLoopSlot slot, int types, PIDValues defaults) {
        this(types, defaults, sparkMax, slot, null, null, null, null);
        checkSupported("SparkMax", types, Type.PID | Type.I_ZONE | Type.LINEAR_FF | Type.COS);
    }

    // PIDController
    public PIDSendable(PIDController pidController, int types, @Nullable PIDValues defaults) {
        this(types, defaults, null, null, pidController, null, null, null);
        checkSupported("PIDController", types, Type.PID | Type.I_ZONE);
    }

    public PIDSendable(PIDController pidController, int types) {
        this(pidController, types, null);
    }
    // PIDController + ElevatorFeedforward
    public PIDSendable(
            PIDController pidController,
            ElevatorFeedforward elevatorFeedforward,
            int types,
            @Nullable PIDValues defaults) {
        this(types, defaults, null, null, pidController, null, elevatorFeedforward, null);
        checkSupported("PIDController/ElevatorFeedforward", types, Type.PID | Type.I_ZONE | Type.LINEAR_FF);
    }

    public PIDSendable(PIDController pidController, ElevatorFeedforward elevatorFeedforward, int types) {
        this(pidController, elevatorFeedforward, types, null);
    }
    // PIDController + ArmFeedforward
    public PIDSendable(
            PIDController pidController, ArmFeedforward armFeedforward, int types, @Nullable PIDValues defaults) {
        this(types, defaults, null, null, pidController, null, null, armFeedforward);
        checkSupported("PIDController/ArmFeedforward", types, Type.PID | Type.I_ZONE | Type.ROTARY_FF);
    }

    public PIDSendable(PIDController pidController, ArmFeedforward armFeedforward, int types) {
        this(pidController, armFeedforward, types, null);
    }

    // ProfiledPIDController
    public PIDSendable(ProfiledPIDController profiledPIDController, int types, @Nullable PIDValues defaults) {
        this(types, defaults, null, null, null, profiledPIDController, null, null);
        checkSupported("ProfiledPIDController", types, Type.PID | Type.I_ZONE | Type.CONSTRAINTS);
    }

    public PIDSendable(ProfiledPIDController profiledPIDController, int types) {
        this(profiledPIDController, types, null);
    }
    // ProfiledPIDController + ElevatorFeedforward
    public PIDSendable(
            ProfiledPIDController profiledPIDController,
            ElevatorFeedforward elevatorFeedforward,
            int types,
            @Nullable PIDValues defaults) {
        this(types, defaults, null, null, null, profiledPIDController, elevatorFeedforward, null);
        checkSupported(
                "ProfiledPIDController/ElevatorFeedforward",
                types,
                Type.PID | Type.I_ZONE | Type.CONSTRAINTS | Type.LINEAR_FF);
    }

    public PIDSendable(
            ProfiledPIDController profiledPIDController, ElevatorFeedforward elevatorFeedforward, int types) {
        this(profiledPIDController, elevatorFeedforward, types, null);
    }
    // ProfiledPIDController + ArmFeedforward
    public PIDSendable(
            ProfiledPIDController profiledPIDController,
            ArmFeedforward armFeedforward,
            int types,
            @Nullable PIDValues defaults) {
        this(types, defaults, null, null, null, profiledPIDController, null, armFeedforward);
        checkSupported(
                "ProfiledPIDController/ArmFeedforward",
                types,
                Type.PID | Type.I_ZONE | Type.CONSTRAINTS | Type.ROTARY_FF);
    }

    public PIDSendable(ProfiledPIDController profiledPIDController, ArmFeedforward armFeedforward, int types) {
        this(profiledPIDController, armFeedforward, types, null);
    }

    // Feedforwards
    public PIDSendable(ElevatorFeedforward elevatorFeedforward, int types, @Nullable PIDValues defaults) {
        this(types, defaults, null, null, null, null, elevatorFeedforward, null);
        checkSupported("ElevatorFeedforward", types, Type.LINEAR_FF);
    }

    public PIDSendable(ElevatorFeedforward elevatorFeedforward, int types) {
        this(elevatorFeedforward, types, null);
    }

    public PIDSendable(ArmFeedforward armFeedforward, int types, @Nullable PIDValues defaults) {
        this(types, defaults, null, null, null, null, null, armFeedforward);
        checkSupported("ArmFeedforward", types, Type.ROTARY_FF);
    }

    public PIDSendable(ArmFeedforward armFeedforward, int types) {
        this(armFeedforward, types, null);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        final var config = sparkMax != null && closedLoopSlot != null ? new SparkMaxConfig() : null;
        if ((type & Type.P) != 0) {
            if (config != null) {
                builder.addDoubleProperty("P", sparkMaxPIDValues::getP, v -> {
                    sparkMaxPIDValues.setP(v);
                    config.closedLoop.p(v, closedLoopSlot);
                });
            } else if (pidController != null) {
                builder.addDoubleProperty("P", pidController::getP, pidController::setP);
            } else if (profiledPIDController != null) {
                builder.addDoubleProperty("P", profiledPIDController::getP, profiledPIDController::setP);
            }
        }
        if ((type & Type.I) != 0) {
            if (config != null) {
                builder.addDoubleProperty("I", sparkMaxPIDValues::getI, v -> {
                    sparkMaxPIDValues.setI(v);
                    config.closedLoop.i(v, closedLoopSlot);
                });
            } else if (pidController != null) {
                builder.addDoubleProperty("I", pidController::getI, pidController::setI);
            } else if (profiledPIDController != null) {
                builder.addDoubleProperty("I", profiledPIDController::getI, profiledPIDController::setI);
            }
        }
        if ((type & Type.D) != 0) {
            if (config != null) {
                builder.addDoubleProperty("D", sparkMaxPIDValues::getD, v -> {
                    sparkMaxPIDValues.setD(v);
                    config.closedLoop.d(v, closedLoopSlot);
                });
            } else if (pidController != null) {
                builder.addDoubleProperty("D", pidController::getD, pidController::setD);
            } else if (profiledPIDController != null) {
                builder.addDoubleProperty("D", profiledPIDController::getD, profiledPIDController::setD);
            }
        }
        if ((type & Type.I_ZONE) != 0) {
            if (config != null) {
                builder.addDoubleProperty("I Zone", sparkMaxPIDValues::getIZone, v -> {
                    sparkMaxPIDValues.setIZone(v);
                    config.closedLoop.iZone(v, closedLoopSlot);
                });
            } else if (pidController != null) {
                builder.addDoubleProperty("I Zone", pidController::getIZone, pidController::setIZone);
            } else if (profiledPIDController != null) {
                builder.addDoubleProperty("I Zone", profiledPIDController::getIZone, profiledPIDController::setIZone);
            }
        }
        if ((type & Type.S) != 0) {
            if (config != null) {
                builder.addDoubleProperty("S", sparkMaxPIDValues::getS, v -> {
                    sparkMaxPIDValues.setS(v);
                    config.closedLoop.feedForward.kS(v, closedLoopSlot);
                });
            } else if (elevatorFeedforward != null) {
                builder.addDoubleProperty("S", elevatorFeedforward::getKs, elevatorFeedforward::setKs);
            } else if (armFeedforward != null) {
                builder.addDoubleProperty("S", armFeedforward::getKs, armFeedforward::setKs);
            }
        }
        if ((type & Type.G) != 0) {
            if (config != null) {
                builder.addDoubleProperty("G", sparkMaxPIDValues::getG, v -> {
                    sparkMaxPIDValues.setG(v);
                    config.closedLoop.feedForward.kG(v, closedLoopSlot);
                });
            } else if (elevatorFeedforward != null) {
                builder.addDoubleProperty("G", elevatorFeedforward::getKg, elevatorFeedforward::setKg);
            }
        }
        if ((type & Type.V) != 0) {
            if (config != null) {
                builder.addDoubleProperty("V", sparkMaxPIDValues::getV, v -> {
                    sparkMaxPIDValues.setV(v);
                    config.closedLoop.feedForward.kV(v, closedLoopSlot);
                });
            } else if (elevatorFeedforward != null) {
                builder.addDoubleProperty("V", elevatorFeedforward::getKv, elevatorFeedforward::setKv);
            } else if (armFeedforward != null) {
                builder.addDoubleProperty("V", armFeedforward::getKv, armFeedforward::setKv);
            }
        }
        if ((type & Type.A) != 0) {
            if (config != null) {
                builder.addDoubleProperty("A", sparkMaxPIDValues::getA, v -> {
                    sparkMaxPIDValues.setA(v);
                    config.closedLoop.feedForward.kA(v, closedLoopSlot);
                });
            } else if (elevatorFeedforward != null) {
                builder.addDoubleProperty("A", elevatorFeedforward::getKa, elevatorFeedforward::setKa);
            } else if (armFeedforward != null) {
                builder.addDoubleProperty("A", armFeedforward::getKa, armFeedforward::setKa);
            }
        }
        if ((type & Type.COS) != 0) {
            if (config != null) {
                builder.addDoubleProperty("COS", sparkMaxPIDValues::getG, v -> {
                    sparkMaxPIDValues.setG(v);
                    config.closedLoop.feedForward.kCos(v, closedLoopSlot);
                });
            } else if (armFeedforward != null) {
                builder.addDoubleProperty("COS", armFeedforward::getKg, armFeedforward::setKg);
            }
        }
        if ((type & Type.MAX_VELOCITY) != 0 && profiledPIDController != null) {
            builder.addDoubleProperty(
                    "Max Velocity",
                    () -> profiledPIDController.getConstraints().maxVelocity,
                    (v) -> profiledPIDController.setConstraints(new TrapezoidProfile.Constraints(
                            v, profiledPIDController.getConstraints().maxAcceleration)));
        }
        if ((type & Type.MAX_ACCELERATION) != 0 && profiledPIDController != null) {
            builder.addDoubleProperty(
                    "Max Acceleration",
                    () -> profiledPIDController.getConstraints().maxAcceleration,
                    (v) -> profiledPIDController.setConstraints(
                            new TrapezoidProfile.Constraints(profiledPIDController.getConstraints().maxVelocity, v)));
        }
        if (sparkMax != null) {
            builder.addDoubleProperty(
                    "Setpoint",
                    () -> sparkMax.getClosedLoopController().getSetpoint(),
                    (v) -> sparkMax.getClosedLoopController()
                            .setSetpoint(v, sparkMax.getClosedLoopController().getControlType(), closedLoopSlot));
            builder.addBooleanProperty(
                    "At Setpoint", () -> sparkMax.getClosedLoopController().isAtSetpoint(), null);
        }
        if (pidController != null) {
            builder.addDoubleProperty("Setpoint", pidController::getSetpoint, pidController::setSetpoint);
            builder.addBooleanProperty("At Setpoint", pidController::atSetpoint, null);
        }
        if (profiledPIDController != null) {
            builder.addDoubleProperty("Setpoint Position", () -> profiledPIDController.getSetpoint().position, null);
            builder.addDoubleProperty("Setpoint Velocity", () -> profiledPIDController.getSetpoint().velocity, null);
            builder.addBooleanProperty("At Setpoint", profiledPIDController::atSetpoint, null);
            builder.addDoubleProperty(
                    "Goal Position",
                    () -> profiledPIDController.getGoal().position,
                    (v) -> profiledPIDController.setGoal(
                            new TrapezoidProfile.State(v, profiledPIDController.getGoal().velocity)));
            builder.addDoubleProperty(
                    "Goal Velocity",
                    () -> profiledPIDController.getGoal().velocity,
                    (v) -> profiledPIDController.setGoal(
                            new TrapezoidProfile.State(profiledPIDController.getGoal().position, v)));
            builder.addBooleanProperty("At Goal", profiledPIDController::atGoal, null);
        }

        if (config != null) {
            Objects.requireNonNull(sparkMax) // NullAway false positive
                    .configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    private void checkSupported(final String name, final int actualTypes, final int supportedTypes) {
        if ((supportedTypes & actualTypes) != actualTypes) {
            final StringBuilder unsupportedTypes = new StringBuilder();
            for (int i = 0; i < Type.VALUES.size(); i++) {
                final int typeBit = 1 << i;
                if ((actualTypes & typeBit) != 0 && (supportedTypes & typeBit) == 0) {
                    if (!unsupportedTypes.isEmpty()) {
                        unsupportedTypes.append(", ");
                    }
                    unsupportedTypes.append(Type.VALUES.get(i));
                }
            }
            throw new IllegalArgumentException(
                    String.format("%s does not support the following types: %s", name, unsupportedTypes));
        }
    }

    @NullMarked
    public static class Type {
        // PID
        public static final int P = 1;
        public static final int I = 1 << 1;
        public static final int D = 1 << 2;
        public static final int I_ZONE = 1 << 3;

        // FeedForward
        public static final int S = 1 << 4; // static gain
        public static final int G = 1 << 5; // static (elevator) gravity gain
        public static final int V = 1 << 6; // velocity gain
        public static final int A = 1 << 7; // acceleration gain
        public static final int COS = 1 << 8; // cosine (arm) gravity gain

        // Constraints
        public static final int MAX_VELOCITY = 1 << 9;
        public static final int MAX_ACCELERATION = 1 << 10;

        public static final int PID = P | I | D;
        public static final int LINEAR_FF = S | G | V | A;
        public static final int ROTARY_FF = S | COS | V | A;
        public static final int CONSTRAINTS = MAX_VELOCITY | MAX_ACCELERATION;

        public static final List<String> VALUES =
                List.of("P", "I", "D", "I Zone", "S", "G", "V", "A", "COS", "Max Velocity", "Max Acceleration");

        public static int getIndexFromType(int type) {
            return Integer.numberOfTrailingZeros(type);
        }

        private Type() {}
    }

    @NullMarked
    public static class PIDValues {
        private double p;
        private double i;
        private double d;
        private double ff;
        private double iZone;
        private double s;
        private double g;
        private double v;
        private double a;
        private double maxVelocity;
        private double maxAcceleration;

        public PIDValues() {
            this(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }

        public static PIDValues pidf(
                double p, double i, double d, double ff, double iZone, double maxVelocity, double maxAcceleration) {
            return new PIDValues(p, i, d, ff, iZone, 0, 0, 0, 0, maxVelocity, maxAcceleration);
        }

        public static PIDValues feedForward(double s, double g, double v, double a) {
            return new PIDValues(0, 0, 0, 0, 0, s, g, v, a, 0, 0);
        }

        public static PIDValues from(PIDController pidController) {
            return pidf(
                    pidController.getP(),
                    pidController.getI(),
                    pidController.getD(),
                    0,
                    pidController.getIZone(),
                    0,
                    0);
        }

        public static PIDValues from(ProfiledPIDController profiledPIDController) {
            return pidf(
                    profiledPIDController.getP(),
                    profiledPIDController.getI(),
                    profiledPIDController.getD(),
                    0,
                    profiledPIDController.getIZone(),
                    profiledPIDController.getConstraints().maxVelocity,
                    profiledPIDController.getConstraints().maxAcceleration);
        }

        public static PIDValues from(ElevatorFeedforward elevatorFeedforward) {
            return feedForward(
                    elevatorFeedforward.getKs(),
                    elevatorFeedforward.getKg(),
                    elevatorFeedforward.getKv(),
                    elevatorFeedforward.getKa());
        }

        public static PIDValues from(ArmFeedforward armFeedforward) {
            return feedForward(
                    armFeedforward.getKs(), armFeedforward.getKg(), armFeedforward.getKv(), armFeedforward.getKa());
        }

        public static PIDValues from(PIDController pidController, ElevatorFeedforward elevatorFeedforward) {
            return from(pidController).and(from(elevatorFeedforward));
        }

        public static PIDValues from(
                ProfiledPIDController profiledPIDController, ElevatorFeedforward elevatorFeedforward) {
            return from(profiledPIDController).and(from(elevatorFeedforward));
        }

        public static PIDValues from(PIDController pidController, ArmFeedforward armFeedforward) {
            return from(pidController).and(from(armFeedforward));
        }

        public static PIDValues from(ProfiledPIDController profiledPIDController, ArmFeedforward armFeedforward) {
            return from(profiledPIDController).and(from(armFeedforward));
        }

        public PIDValues(
                double p,
                double i,
                double d,
                double ff,
                double iZone,
                double s,
                double g,
                double v,
                double a,
                double maxVelocity,
                double maxAcceleration) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.ff = ff;
            this.iZone = iZone;
            this.s = s;
            this.g = g;
            this.v = v;
            this.a = a;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }

        public PIDValues copy() {
            return new PIDValues(p, i, d, ff, iZone, s, g, v, a, maxVelocity, maxAcceleration);
        }

        /**
         * Combines the current PIDValues instance with another instance by selecting non-zero values from each
         * corresponding field. If both values for a field are non-zero, an IllegalArgumentException is thrown.
         *
         * @param other The other PIDValues instance to combine with this instance.
         * @return A new PIDValues instance containing the combined values.
         * @throws IllegalArgumentException If both this instance and the other instance contain non-zero values for any
         *     field.
         */
        public PIDValues and(PIDValues other) {
            return new PIDValues(
                    getOneValueOrThrow(this.p, other.p, "P"),
                    getOneValueOrThrow(this.i, other.i, "I"),
                    getOneValueOrThrow(this.d, other.d, "D"),
                    getOneValueOrThrow(this.ff, other.ff, "FF"),
                    getOneValueOrThrow(this.iZone, other.iZone, "I Zone"),
                    getOneValueOrThrow(this.s, other.s, "S"),
                    getOneValueOrThrow(this.g, other.g, "G"),
                    getOneValueOrThrow(this.v, other.v, "V"),
                    getOneValueOrThrow(this.a, other.a, "A"),
                    getOneValueOrThrow(this.maxVelocity, other.maxVelocity, "Max Velocity"),
                    getOneValueOrThrow(this.maxAcceleration, other.maxAcceleration, "Max Acceleration"));
        }

        private double getOneValueOrThrow(double value1, double value2, String valueName) {
            if (value1 == 0) return value2;
            if (value2 == 0) return value1;
            throw new IllegalArgumentException(
                    String.format("PID Value %s cannot be both %s and %s", valueName, value1, value2));
        }

        public double getP() {
            return p;
        }

        public void setP(double p) {
            this.p = p;
        }

        public double getI() {
            return i;
        }

        public void setI(double i) {
            this.i = i;
        }

        public double getD() {
            return d;
        }

        public void setD(double d) {
            this.d = d;
        }

        public double getFf() {
            return ff;
        }

        public void setFf(double ff) {
            this.ff = ff;
        }

        public double getIZone() {
            return iZone;
        }

        public void setIZone(double iZone) {
            this.iZone = iZone;
        }

        public double getS() {
            return s;
        }

        public void setS(double s) {
            this.s = s;
        }

        public double getG() {
            return g;
        }

        public void setG(double g) {
            this.g = g;
        }

        public double getV() {
            return v;
        }

        public void setV(double v) {
            this.v = v;
        }

        public double getA() {
            return a;
        }

        public void setA(double a) {
            this.a = a;
        }

        public double getMaxVelocity() {
            return maxVelocity;
        }

        public void setMaxVelocity(double maxVelocity) {
            this.maxVelocity = maxVelocity;
        }

        public double getMaxAcceleration() {
            return maxAcceleration;
        }

        public void setMaxAcceleration(double maxAcceleration) {
            this.maxAcceleration = maxAcceleration;
        }
    }
}
