package org.team1502.configuration.builders;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.Function;

import org.team1502.configuration.MdFormatter;
import org.team1502.configuration.CAN.CanInfo;
import org.team1502.configuration.CAN.CanMap;
import org.team1502.configuration.factory.PartBuilder;

public class TestBuilder implements IBuild {

    private ArrayList<Part> _parts = new ArrayList<>(); // every part created
    
    //** source of IBuild for testing */
    public TestBuilder(){}
    
    //** use for reporting on already created parts */
    public TestBuilder(ArrayList<Part> parts){ _parts = parts; }

    public Builder createPart(String partName, Function<Builder, Builder> fn) {
        var part = createPart(partName);
        fn.apply(part);
        return part;
    }

    public Builder createPart(String partName) { return Builder.DefineAs(partName).apply(this);  }

    @Override
    public <T extends Builder> PartBuilder<T> getTemplate(String partName, Function<T, Builder> buildFunction) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTemplate'");
    }
    @Override
    public <T extends Builder> PartBuilder<T> getTemplate(String partName, Function<IBuild, T> createFunction,
            Function<T, Builder> buildFunction) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTemplate'");
    }

    @Override
    public void register(Part part) { _parts.add(part); }

    @Override
    public Builder getInstalled(String name) {
        for (Part part : _parts) {
            if (part.getName() == name) {
                return new Builder(this, part);
            }
        }
        return null;
    }

    public void DumpParts() {
        var keySet = new HashSet<String>();


        var formatter = MdFormatter.Table( _parts.size() + " Parts Registered")
            .Heading("name", "template", "class", "Key", "Friendly Name", "Label");

        for (Part part : _parts) {
            var key = part.getKey();
            if (!keySet.add(key)) {
                System.out.println(key + " is a duplicate");
            }
            formatter.AddRow(
                part.hasValue(Part.BUILD_NAME) ? (String)part.getValue(Part.BUILD_NAME) : "",
                part.hasValue(Part.TEMPLATE_NAME) ? (String)part.getValue(Part.TEMPLATE_NAME) : "",
                part.hasValue(Part.CLASS_NAME) ? (String)part.getValue(Part.CLASS_NAME) : "",
                part.getKey(),
                part.hasValue(Part.friendlyName) ? (String)part.getValue(Part.friendlyName) : "",
                part.hasValue(Connector.label) ? (String)part.getValue(Connector.label) : ""
            );
            if (part.getErrors().size() != 0) {
                formatter.AddRow(part.getErrors().get(0));
            }
        }
        formatter.PrintTable();

    }
    public void showType(String className) {
        var parts = _parts.stream()
            .filter(part->((String)part.getValue(Part.CLASS_NAME)) == className)
            .toList();

        var formatter = MdFormatter.Table( _parts.size() + " Parts Registered")
            .Heading("name", "template", "class", "Key", "Friendly Name", "Label");

        for (Part part : parts) {
            formatter.AddRow(
                part.hasValue(Part.BUILD_NAME) ? (String)part.getValue(Part.BUILD_NAME) : "",
                part.hasValue(Part.TEMPLATE_NAME) ? (String)part.getValue(Part.TEMPLATE_NAME) : "",
                part.hasValue(Part.CLASS_NAME) ? (String)part.getValue(Part.CLASS_NAME) : "",
                part.getKey(),
                part.hasValue(Part.friendlyName) ? (String)part.getValue(Part.friendlyName) : "",
                part.hasValue(Connector.label) ? (String)part.getValue(Connector.label) : ""
            );
        }
        formatter.PrintTable();

    }

    public void reportPartPower() {
        var formatter = MdFormatter.Table("Parts Power")
            .Heading("Watts", "Total", "Key", "Friendly Name", "Label");

        var parts = _parts.stream().map(p->new Builder(this, p)).toList();
        for (Builder part : parts) {
            var key = part.getPart().getKey();
            formatter.AddRow(
                part.hasPowerProfile() ? part.PowerProfile().PeakPower().toString() : "",
                part.hasValue("totalPeakPower") ? part.getDouble("totalPeakPower").toString() : "",
                key,
                part.hasValue(Part.friendlyName) ? (String)part.getValue(Part.friendlyName) : "",
                part.hasValue(Connector.label) ? (String)part.getValue(Connector.label) : ""
            );
        }
        formatter.PrintTable();
    }
    
    public void reportUnconnected() {
        var connectors = Connector.findConnectors(_parts);
        var unconnected = connectors.stream().filter(c->!c.isConnected()).toList();
        if (!unconnected.isEmpty()) {
            var formatter = MdFormatter.Table("WARNING: Unconnected connectors")
            .Heading("Part", "Friendly", "Signal", "Host");

           for (Connector part : unconnected) {
            formatter.AddRow(
                part.getPart().getKey(),
                part.FriendlyName(),
                part.Signal(),
                part.Host().FriendlyName()
                );    
            }
            formatter.PrintTable();

        }
    }

    public void reportCanBus(Builder bus) {
        var map = new CanMap(bus);

        var formatter = MdFormatter.Table("Channels for " + bus.Name())
            .Heading("Part", "Friendly", "Number", "Device", "Problem");

        for (Builder part : map.getDevices()) {
            var caninfo = CanInfo.WrapPart(part);
            formatter.AddRow(
                part.getPart().getKey(),
                part.FriendlyName(),
                caninfo.Number().toString(),
                caninfo.FriendlyName(),
                part.hasErrors() ? part.getErrors().get(0) : ""
            );    
        }
        formatter.PrintTable();
    }

    public void DumpChannels(Builder hub) {
       var partChannels = Channel.getPartChannels(hub);
       var pieceChannels = Channel.getPieceChannels(hub);

       var formatter = MdFormatter.Table("Channels for " + hub.Name())
        .Heading("Part", "Friendly", "Signal", "Network", "Connection");

       for (Channel part : partChannels) {
        formatter.AddRow(
            part.getPart().getKey(),
            part.FriendlyName(),
            part.Signal(),
            part.Network(),
            part.isConnected() ? part.Connection().getPart().getKey() : ""
            );    
        }
       for (Channel part : pieceChannels) {
        formatter.AddRow(
            part.getPart().getKey(),
            part.FriendlyName(),
            part.Signal(),
            part.Network(),
            part.isConnected() ? part.Connection().getPart().getKey() : ""
            );    
        }
        formatter.PrintTable();
    }

}
