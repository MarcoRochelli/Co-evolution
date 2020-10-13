package it.units.erallab;

import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;

import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import static java.lang.Math.*;

public class PrintBodies {

  public static <K> String toString(Grid<K> grid, Predicate<K> p) {
    StringBuilder sb = new StringBuilder();
    for (int y = 0; y < grid.getH(); y++) {
      for (int x = 0; x < grid.getW(); x++) {
        sb.append(p.test(grid.get(x, y)) ? "1" : "0");
      }
      if (y < grid.getH() - 1) {
        sb.append(String.format("/"));
      }
    }
    return sb.toString();
  }


  public static void main(String[] args) {
    Random random = new Random();
    final int width = 10;
    final int height = 10;
    final double THRESHOLD = 0;

    // bit representation
    for (int i = 0; i < 10; i++) {
      //generate random body
      UniformDoubleFactory udf = new UniformDoubleFactory(-1, 1);
      FixedLengthListFactory<Double> factory = new FixedLengthListFactory<>(width * height, udf);
      List<Double> genotype = factory.build(random);
      int c = 0;
      Grid<String> body = Grid.create(width, height);
      for (double entry : genotype) {
        if (c < width * height) {
          if (entry > THRESHOLD) {
            body.set(c % width, c / width, "x");
          } else {
            body.set(c % width, c / width, " ");
          }
          c = c + 1;
        }
      }
      // creates an empty grid
      Grid<String> emptyBody = Grid.create(body.getW(), body.getH(), (x, y) -> " ");
      // checks if the robot is empty
      if (body.equals(emptyBody)) {
        body.set(0, 0, "x");
      }

      // checks if the robot is connected
      body = Utils.gridLargestConnected(body, x -> !x.equals(" "));
      for (Grid.Entry<String> entry : body) {
        if (entry.getValue() == null) {
          body.set(entry.getX(), entry.getY(), " ");
        }
      }

      // print body
      System.out.println("bit representation");
      System.out.println(Grid.toString(body, "%s"));
      System.out.println();


      // gaussian representation
      GaussianFactory<Double> gaussianFactory = new GaussianFactory<>(25, 5); // length must be 5*nOfGaussian
      List<Double> gaussianGenotype = gaussianFactory.build(random);
      Grid<Double> values = Grid.create(width, height);
      for (Grid.Entry<Double> entry : values) {
        double value = 0;
        for (int l = 0; l < 5; l++) {
          double weight = gaussianGenotype.get(5 * l);
          double Ux = gaussianGenotype.get(1 + 5 * l);
          double Uy = gaussianGenotype.get(2 + 5 * l);
          double Ox = gaussianGenotype.get(3 + 5 * l);
          double Oy = gaussianGenotype.get(4 + 5 * l);
          // normalization
          double x = (double) entry.getX() / (double) width;
          double y = (double) entry.getY() / (double) height;
          value += weight * (exp(-0.5 * (pow((x - Ux), 2.0) / pow(Ox, 2.0) + pow((y - Uy), 2.0) / pow(Oy, 2.0))) / (2 * PI * Ox * Oy));
        }
        values.set(entry.getX(), entry.getY(), value);
      }

      Grid<String> gaussianBody = Grid.create(width, height);
      // for each value of values if it is bigger tha a threshold puts a voxel in that position
      for (Grid.Entry<Double> ent : values) {
        if (ent.getValue() > THRESHOLD) {
          gaussianBody.set(ent.getX(), ent.getY(), "x");
        } else {
          gaussianBody.set(ent.getX(), ent.getY(), " ");
        }
      }

      if (gaussianBody.equals(emptyBody)) {
        gaussianBody.set(0, 0, "x"); // if the body is empty puts a voxel (0,0)
      }

      // checks if the robot is connected
      gaussianBody = Utils.gridLargestConnected(gaussianBody, x -> !x.equals(" "));
      for (Grid.Entry<String> entry : gaussianBody) {
        if (entry.getValue() == null) {
          gaussianBody.set(entry.getX(), entry.getY(), " ");
        }
      }

      // print body
      System.out.println("gaussian representation");
      System.out.println(Grid.toString(gaussianBody, "%s"));
      System.out.println();
    }
  }

}
