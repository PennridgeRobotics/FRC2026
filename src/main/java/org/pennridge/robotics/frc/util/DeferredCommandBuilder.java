package org.pennridge.robotics.frc.util;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class DeferredCommandBuilder {
    private @NotNull Supplier<@Nullable Command> backingCommand = () -> null;
    private @NotNull Function<@NotNull Command, @NotNull Set<@NotNull Subsystem>> getRequirements = (cmd) -> Set.of();
    private @NotNull Consumer<@NotNull Command> initialize = (cmd) -> {};
    private @NotNull Consumer<@NotNull Command> execute = (cmd) -> {};
    private @NotNull Function<@NotNull Command, @NotNull Boolean> isFinished = (cmd) -> false;
    private @NotNull BiConsumer<@NotNull Command, @NotNull Boolean> end = (cmd, interrupted) -> {};
    private @NotNull BiConsumer<@NotNull Command, @NotNull SendableBuilder> initSendable = (cmd, sendable) -> {};

    public @NotNull DeferredCommandBuilder setBackingCommand(
            final @NotNull Supplier<@Nullable Command> backingCommand) {
        this.backingCommand = backingCommand;
        return this;
    }

    public @NotNull DeferredCommandBuilder setInitialize(final @NotNull Consumer<@NotNull Command> initialize) {
        this.initialize = initialize;
        return this;
    }

    public @NotNull DeferredCommandBuilder setExecute(final @NotNull Consumer<@NotNull Command> execute) {
        this.execute = execute;
        return this;
    }

    public @NotNull DeferredCommandBuilder setIsFinished(
            final @NotNull Function<@NotNull Command, @NotNull Boolean> isFinished) {
        this.isFinished = isFinished;
        return this;
    }

    public @NotNull DeferredCommandBuilder setEnd(final @NotNull BiConsumer<@NotNull Command, @NotNull Boolean> end) {
        this.end = end;
        return this;
    }

    public @NotNull DeferredCommandBuilder setInitSendable(
            final @NotNull BiConsumer<@NotNull Command, @NotNull SendableBuilder> initSendable) {
        this.initSendable = initSendable;
        return this;
    }

    public @NotNull DeferredCommandBuilder setSubsystems(final @NotNull Set<@NotNull Subsystem> subsystems) {
        this.getRequirements = (cmd) -> subsystems;
        return this;
    }

    public @NotNull DeferredCommandBuilder setGetRequirements(
            final @NotNull Function<@NotNull Command, @NotNull Set<@NotNull Subsystem>> getRequirements) {
        this.getRequirements = getRequirements;
        return this;
    }

    public @NotNull Command buildCommand() {
        return new Command() {
            private @NotNull Command currentBackingCommand = Commands.none();

            @Override
            public void initialize() {
                currentBackingCommand = Objects.requireNonNullElse(backingCommand.get(), Commands.none());
                currentBackingCommand.initialize();
                initialize.accept(this);
            }

            @Override
            public void execute() {
                currentBackingCommand.execute();
                execute.accept(this);
            }

            @Override
            public void end(boolean interrupted) {
                currentBackingCommand.end(interrupted);
                end.accept(this, interrupted);
            }

            @Override
            public boolean isFinished() {
                return currentBackingCommand.isFinished() || isFinished.apply(this);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                final var requirements = new HashSet<>(currentBackingCommand.getRequirements());
                requirements.addAll(getRequirements.apply(this));
                return requirements;
            }

            @Override
            public void initSendable(SendableBuilder builder) {
                super.initSendable(builder);
                currentBackingCommand.initSendable(builder);
                initSendable.accept(this, builder);
            }
        };
    }
}
