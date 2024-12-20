package org.team1502.configuration.builders;

import java.util.function.Function;

import org.team1502.configuration.factory.PartBuilder;

public interface IBuild {
    <T extends Builder> PartBuilder<T> getTemplate(String partName, Function<IBuild, T> createFunction, Function<T, Builder> buildFunction);
    void register(Part part);
    Builder getInstalled(String name);
}
