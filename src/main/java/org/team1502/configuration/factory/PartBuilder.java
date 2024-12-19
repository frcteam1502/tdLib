package org.team1502.configuration.factory;

import java.util.function.Function;

import org.team1502.configuration.builders.Builder;
import org.team1502.configuration.builders.IBuild;
import org.team1502.configuration.builders.Part;

/** Use PartBuilder when possible to follow the creation process */
public class PartBuilder<T extends Builder>  {
    /** a standard build template */
    public PartBuilder(String partName, Function<IBuild, T> createFunction, Function<T, Builder> buildFunction) {
        this.partName = partName;
        this.createFunction = createFunction;
        this.buildFunction = buildFunction;
    }

    // e.g., addPart/addPiece -- create without an exisitng template
    public PartBuilder(Function<IBuild, T> createFunction, Function<T, Builder> buildFunction) {
        this.createFunction = createFunction;
        this.buildFunction = buildFunction;
    }

    public PartBuilder<T> with(Function<T, Builder> modifyFunction) {
        return new PartBuilder<T>(this, modifyFunction);
    }
    private PartBuilder(PartBuilder<T> partBuilder, Function<T, Builder> modifyFunction) {
        this.templateBuilder = partBuilder;
        this.buildFunction = modifyFunction;
    }

    // getTemplate() parameters -- i.e., a PartBuilder
    
    /** this is the intended/default name from the template */
    private String partName;
    /** this function creates the builder-of-part when there is no template (templateBuilder) */
    private Function<IBuild, T> createFunction;
    /** additional modifications to part after created */
    private Function<T, Builder> buildFunction;

    /** this is a template for this part, takes precedence ovr createFunction */
    private PartBuilder<T> templateBuilder;
    
    //this part must be named and parented and registered before the buildFunction
    public T addBuilder(Builder parent) { return addBuilder(parent, null); }

    public T addBuilder(Builder parent, String name) {
        T builder = null;
        if (templateBuilder != null) {
            builder = templateBuilder.addBuilder(parent, name);
        } else {
            builder = createFunction.apply(parent.getIBuild());
            if (partName != null) {
                builder.Value(Part.TEMPLATE_NAME, partName);
            }    
            // part is registered in the Builder constructor
            if (name != null) {
                builder.Value(Part.BUILD_NAME, name);
            }
            builder.getPart().setParent(parent.getPart());
            builder.initialize();
        }
        return build(builder);
    }
    
    /** 1. create builder-of-part (from template or createFunction)
        2. build part with (optional) buildFunction
     */
    public T createBuilder(IBuild build, String name) {
        T builder = null;
        if (templateBuilder != null) {
            builder = templateBuilder.createBuilder(build, name);
        } else {
            builder = createFunction.apply(build);
            if (partName != null) {
                builder.Value(Part.TEMPLATE_NAME, partName);
            }    
            // part is registered in the Builder constructor
            if (name != null) {
                builder.Value(Part.BUILD_NAME, name);
            }
            builder.initialize();
        }
        return build(builder);
    }
    
    private T build(T builder) {
        if (buildFunction != null) { // may just need a "Define"
            buildFunction.apply(builder);
        }
        return builder;
    }

}
