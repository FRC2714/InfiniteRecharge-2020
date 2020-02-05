package frc.robot.utils;

public class Tests {
    public static void testInterpolatingTreeMap() {
        InterpolatingTreeMap imap = new InterpolatingTreeMap();

        double dx = 0.1;

        for (int i = 0; i <= 50; i++) {
            imap.put(i * dx, Math.sin(i * dx));
        }

        System.out.println("sin(30) " + imap.getInterpolated(Math.toRadians(30)));
        System.out.println("sin(60) " + imap.getInterpolated(Math.toRadians(60)));
        System.out.println("sin(90) " + imap.getInterpolated(Math.toRadians(90)));
    }
}
