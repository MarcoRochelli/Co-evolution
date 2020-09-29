package it.units.erallab;


import it.units.malelab.jgea.core.IndependentFactory;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.collections.ListUtils;

import java.util.Collections;
import java.util.List;
import java.util.Random;


public class GaussianFactory<T> implements IndependentFactory<List<T>> {
  private final int length;
  private final int nOfGaussians;

  public GaussianFactory(int length, int nOfGaussians) {
    this.length = length;
    this.nOfGaussians = nOfGaussians;
  }

  @Override
  public List<T> build(Random random) {

    UniformDoubleFactory weightUdf = new UniformDoubleFactory(-1, 1);
    UniformDoubleFactory bodyUdf = new UniformDoubleFactory(0, 1);
    UniformDoubleFactory controllerUdf = new UniformDoubleFactory(-1, 1);


    // weights of the body
    List<Double> body =  Collections.emptyList() ;
    for (int i = 0; i < nOfGaussians; i++) {
      // each 5 weights first must be between -1 and 1 while others between 0 and 1
      FixedLengthListFactory<Double> weightFactory = new FixedLengthListFactory<>(1, weightUdf);
      FixedLengthListFactory<Double> paramsFactory = new FixedLengthListFactory<>(4, bodyUdf);

      List<Double> weight = weightFactory.build(random);
      List<Double> params = paramsFactory.build(random);
      List<Double> b = ListUtils.union(weight, params);

      // creates a part of the body at every iteration
      body = ListUtils.union(body, b);
    }

    // weights of the neural network
    FixedLengthListFactory<Double> controllerFactory = new FixedLengthListFactory<>(length - 5 * nOfGaussians, controllerUdf);
    List<Double> controller = controllerFactory.build(random);

    return ListUtils.union(body, controller);
  }
}
