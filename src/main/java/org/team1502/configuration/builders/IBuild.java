package org.team1502.configuration.builders;

import java.util.function.Function;

import org.team1502.configuration.factory.PartBuilder;

public interface IBuild {
    /** get a template with additional build function
     * @param <T>
     * @param partName must exist
     * @param buildFunction modifications for the template
     * @return
     */
    <T extends Builder> PartBuilder<T> getTemplate(String partName, Function<T, Builder> buildFunction);
    /** get (or create) a template with additional build function
     * @param <T>
     * @param partName if not fuond, then create a new one
     * @param createFunction initial creation of the template
     * @param buildFunction modifications for the template
     * @return
     */
    <T extends Builder> PartBuilder<T> getTemplate(String partName, Function<IBuild, T> createFunction, Function<T, Builder> buildFunction);
    void register(Part part);
    Builder getInstalled(String name);
}
