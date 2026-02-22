package com.example.visualizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.tinfour.common.IIncrementalTin;
import org.tinfour.common.Vertex;
import org.tinfour.interpolation.IInterpolatorOverTin;
import org.tinfour.interpolation.NaturalNeighborInterpolator;
import org.tinfour.standard.IncrementalTin;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLayeredPane;
import javax.swing.JPanel;
import javax.swing.WindowConstants;

public class Visualizer {
    static double[][] genGrid(IInterpolatorOverTin interp, int dim) {
        double[][] grid = new double[dim][dim];
        double minIn = -72;
        double maxIn = -minIn;
        double totalIn = maxIn - minIn;
        double squareSize = totalIn / dim;

        double maxResult = Integer.MIN_VALUE;
        double minResult = Integer.MAX_VALUE;

        // interpolate all squares
        for (int x = 0; x < grid.length; x++) {
            for (int y = 0; y < grid[x].length; y++) {
                double xIn = squareSize * (x + 0.5) + minIn;
                double yIn = squareSize * ((grid[x].length - y) - 0.5) + minIn;
                // grid[x][y] = Math.sqrt(xIn * xIn + yIn * yIn);
                grid[x][y] = interp.interpolate(xIn, yIn, null);
                if (Double.isFinite(grid[x][y])) {
                    maxResult = Math.max(maxResult, grid[x][y]);
                    minResult = Math.min(minResult, grid[x][y]);
                }
            }
        }

        // normalize between 0-1
        double interpRange = maxResult - minResult;
        for (int x = 0; x < grid.length; x++) {
            for (int y = 0; y < grid[x].length; y++) {
                grid[x][y] = (grid[x][y] - minResult) / interpRange;
            }
        }

        return grid;
    }

    static Color interpColor(double val) {
        // val = 1 - val;
        return new Color(Color.HSBtoRGB(
                (float) (val * 0.25 + 0.9),
                (float) Math.sqrt((1 - val) * 0.4 + 0.09),
                (float) Math.sqrt(val * 0.63 + 0.01)
        ));
    }

    static double[][] grid;

    public static void main(String[] args) {
        double[][] launcherParamsByPos = {
                // x, y, rpm, hood, offset from center (positive is horz, negative is vert)
                // @formatter:off
                // near
                { -37,   46, 1950, 0.0,   0},
                { -19,   31, 2170, 0.3,  -5},
                {  12,    0, 2170, 0.7,  -9},
                { -15,  -29, 2770, 0.6,  -6},
                { -51,  -53, 3100, 1.0,   0},
                { -63,  -38, 3040, 1.0,   0},
                { -65,    0, 2550, 0.9,   2},
                { -65,   29, 2060, 0.0,   4},

                // far
                {  65,  -35, 3420, 1.0,  -8},
                {  34,   -3, 2980, 1.0, -12},
                {  66,   32, 3160, 1.0,  -6},
                {  65,    3, 3270, 1.0,  -5},
                // @formatter:on
        };

        Function<Integer, IInterpolatorOverTin> mkInterp = idx -> {
            List<Vertex> vertices = Arrays.stream(launcherParamsByPos)
                    .map(xs -> new Vertex(xs[0], xs[1], xs[idx]))
                    .collect(Collectors.toList());
            IIncrementalTin tin = new IncrementalTin();
            tin.add(vertices, null);
            return new NaturalNeighborInterpolator(tin);
        };
        IInterpolatorOverTin interpolator = mkInterp.apply(2);

        JFrame frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.setSize(800, 800);
        frame.setResizable(false);

        int sqWidth = 400;

        grid = genGrid(interpolator, sqWidth);
        JPanel map = new JPanel() {
            @Override
            public void paintComponent(Graphics graphics) {
                super.paintComponent(graphics);
                Graphics2D g = (Graphics2D) graphics;

                double rectWidth = (double) this.getWidth() / grid.length;
                double rectHeight = (double) this.getHeight() / grid[0].length;
                for (int x = 0; x < grid.length; x++) {
                    for (int y = 0; y < grid.length; y++) {
                        double value = grid[x][y];
                        if (Double.isFinite(value)) {
                            g.setColor(interpColor(value));
                        } else {
                            g.setColor(Color.GRAY);
                        }
                        g.fillRect(
                                (int) (x * rectWidth),
                                (int) (y * rectHeight),
                                (int) Math.ceil(rectWidth),
                                (int) Math.ceil(rectHeight)
                        );
                    }
                }
            }
        };
        map.setLayout(new BorderLayout());

        JPanel windowPanel = new JPanel();
        windowPanel.setOpaque(false);
        windowPanel.setLayout(new BorderLayout());

        JPanel buttonPanel = new JPanel();
        buttonPanel.setOpaque(false);
        buttonPanel.setLayout(new FlowLayout(FlowLayout.RIGHT));

        JButton rpmButton = new JButton("rpm");
        rpmButton.addActionListener(_e -> {
            IInterpolatorOverTin newInterp = mkInterp.apply(2);
            grid = genGrid(newInterp, sqWidth);
            map.repaint();
        });
        JButton hoodButton = new JButton("hood");
        hoodButton.addActionListener(_e -> {
            IInterpolatorOverTin newInterp = mkInterp.apply(3);
            grid = genGrid(newInterp, sqWidth);
            map.repaint();
        });
        JButton offsetButton = new JButton("offset");
        offsetButton.addActionListener(_e -> {
            IInterpolatorOverTin newInterp = mkInterp.apply(4);
            grid = genGrid(newInterp, sqWidth);
            map.repaint();
        });
        buttonPanel.add(rpmButton);
        buttonPanel.add(hoodButton);
        buttonPanel.add(offsetButton);

        windowPanel.add(buttonPanel, BorderLayout.SOUTH);
        map.add(windowPanel, BorderLayout.CENTER);

        frame.setContentPane(map);
        frame.setVisible(true);

        Pose2d pose = new Pose2d(3, 6, Math.toRadians(30));
        System.out.println(pose.plus(new Twist2d(new Vector2d(5, 0), 0)));
    }
}