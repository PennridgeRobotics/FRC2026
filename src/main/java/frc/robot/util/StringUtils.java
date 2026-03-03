package frc.robot.util;

import org.jspecify.annotations.NullMarked;

@NullMarked
public class StringUtils {
    public static String capitalizeFully(final String original) {
        final char[] chars = original.replace('_', ' ').toLowerCase().toCharArray();
        chars[0] = Character.toUpperCase(chars[0]);
        for (int i = 1; i < chars.length - 1; i++) {
            if (Character.isSpaceChar(chars[i])) {
                chars[i + 1] = Character.toUpperCase(chars[i + 1]);
            }
        }
        return new String(chars);
    }
}
