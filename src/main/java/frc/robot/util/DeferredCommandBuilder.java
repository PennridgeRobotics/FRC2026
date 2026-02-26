package frc.robot.util;

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
import org.jspecify.annotations.NullMarked;
import org.jspecify.annotations.Nullable;

@NullMarked
public class DeferredCommandBuilder {
    private Supplier<@Nullable Command> backingCommand = () -> null;
    private Function<Command, Set<Subsystem>> getRequirements = (cmd) -> Set.of();
    private Consumer<Command> initialize = (cmd) -> {};
    private Consumer<Command> execute = (cmd) -> {};
    private Function<Command, Boolean> isFinished = (cmd) -> false;
    private BiConsumer<Command, Boolean> end = (cmd, interrupted) -> {};
    private BiConsumer<Command, SendableBuilder> initSendable = (cmd, sendable) -> {};

    public DeferredCommandBuilder setBackingCommand(final Supplier<@Nullable Command> backingCommand) {
        this.backingCommand = backingCommand;
        return this;
    }

    public DeferredCommandBuilder setInitialize(final Consumer<Command> initialize) {
        this.initialize = initialize;
        return this;
    }

    public DeferredCommandBuilder setExecute(final Consumer<Command> execute) {
        this.execute = execute;
        return this;
    }

    public DeferredCommandBuilder setIsFinished(final Function<Command, Boolean> isFinished) {
        this.isFinished = isFinished;
        return this;
    }

    public DeferredCommandBuilder setEnd(final BiConsumer<Command, Boolean> end) {
        this.end = end;
        return this;
    }

    public DeferredCommandBuilder setInitSendable(final BiConsumer<Command, SendableBuilder> initSendable) {
        this.initSendable = initSendable;
        return this;
    }

    public DeferredCommandBuilder setSubsystems(final Set<Subsystem> subsystems) {
        this.getRequirements = (cmd) -> subsystems;
        return this;
    }

    public DeferredCommandBuilder setGetRequirements(final Function<Command, Set<Subsystem>> getRequirements) {
        this.getRequirements = getRequirements;
        return this;
    }

    public Command buildCommand() {
        return new Command() {
            private Command currentBackingCommand = Commands.none();

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
