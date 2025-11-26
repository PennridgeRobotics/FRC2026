package org.pennridge.robotics.frc.util.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class SplitButtonChooser<T> implements Sendable {
    private final @NotNull Function<@NotNull String, @NotNull T> stringToType;
    private final @NotNull Function<@NotNull T, @NotNull String> typeToString;
    private final @NotNull Supplier<@Nullable T> active;
    private final @NotNull Collection<@NotNull String> options;
    private final @NotNull Set<@NotNull Consumer<@NotNull T>> listeners;
    private @Nullable String defaultOption;
    private @Nullable String selected;

    public SplitButtonChooser(
            final @Nullable Supplier<@Nullable T> active,
            final @Nullable Collection<@NotNull T> options,
            final @Nullable Set<@NotNull Consumer<@NotNull T>> listeners,
            final @Nullable T defaultOption,
            final @NotNull Function<@NotNull String, @NotNull T> stringToType,
            final @NotNull Function<@NotNull T, @NotNull String> typeToString) {
        this.active = Objects.requireNonNullElse(active, this::getSelected);
        this.options = Objects.requireNonNullElse(options, new ArrayList<T>()).stream()
                .map(typeToString)
                .collect(Collectors.toList());
        this.listeners = Objects.requireNonNullElse(listeners, Collections.newSetFromMap(new ConcurrentHashMap<>()));
        this.defaultOption = defaultOption == null ? null : typeToString.apply(defaultOption);
        this.stringToType = stringToType;
        this.typeToString = typeToString;
    }

    public static @NotNull SplitButtonChooser<String> withStrings(
            final @Nullable Supplier<@Nullable String> active,
            final @Nullable Collection<@NotNull String> options,
            final @Nullable Set<@NotNull Consumer<@NotNull String>> listeners,
            final @Nullable String defaultOption) {
        return new SplitButtonChooser<>(
                active, options, listeners, defaultOption, Function.identity(), Function.identity());
    }

    public void addOption(final @NotNull T option) {
        final String string = typeToString.apply(option);
        if (!options.contains(string)) {
            options.add(string);
        }
    }

    public void setDefaultOption(final @NotNull T defaultOption) {
        this.defaultOption = typeToString.apply(defaultOption);
        addOption(defaultOption);
    }

    public @Nullable T getSelected() {
        if (selected != null) {
            return stringToType.apply(selected);
        }
        return defaultOption == null ? null : stringToType.apply(defaultOption);
    }

    public void onChange(final @NotNull Consumer<@NotNull T> listener) {
        listeners.add(listener);
    }

    @Override
    public void initSendable(final SendableBuilder builder) {
        builder.setSmartDashboardType("Split Button Chooser");
        builder.addStringProperty("default", () -> Objects.requireNonNullElse(defaultOption, ""), null);
        builder.addStringArrayProperty("options", () -> options.toArray(new String[0]), null);
        builder.addStringProperty(
                "active",
                () -> {
                    final T activeValue = active.get();
                    return activeValue == null ? null : typeToString.apply(activeValue);
                },
                null);
        builder.addStringProperty("selected", null, newValue -> {
            final String before = selected;
            selected = newValue;
            if (!Objects.equals(before, newValue)) {
                listeners.forEach(listener -> listener.accept(stringToType.apply(newValue)));
            }
        });
    }
}
