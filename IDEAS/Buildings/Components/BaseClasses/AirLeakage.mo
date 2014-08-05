within IDEAS.Buildings.Components.BaseClasses;
model AirLeakage "air leakage due to limied air tightness"

extends IDEAS.Fluid.Interfaces.PartialTwoPortInterface(
  final allowFlowReversal=false,
  redeclare package Medium = IDEAS.Media.Air,
  m_flow_nominal = V/3600*n50/20);

  outer IDEAS.SimInfoManager sim "Simulation information manager"
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

  parameter Modelica.SIunits.Volume V "zone air volume";
  parameter Real n50(min=0.01)=0.4 "n50-value of airtightness";
  parameter Boolean linear = true
    "Set to false to assume default humidity when calculating the enthalpy";
  Modelica.SIunits.MassFraction Xi[Medium.nXi]
    "Species concentration of the fluid";

equation
  //mass flow rate
  port_a.m_flow+port_b.m_flow=0;
  port_a.m_flow=m_flow_nominal;

  //enthalpy
  port_a.h_outflow=port_b.h_outflow;
  port_a.h_outflow= Medium.specificEnthalpy(Medium.setState_pTX(
                      Medium.p_default,
                      sim.Te,
                      if linear then Medium.X_default else cat(1,Xi,{1-sum(Xi)})));

  Xi=sim.XiEnv.X[1:Medium.nXi];
  // species concentrations
  port_b.Xi_outflow=port_a.Xi_outflow;
  port_b.Xi_outflow=Xi;

  port_a.C_outflow=inStream(port_b.C_outflow);
  port_b.C_outflow=inStream(port_a.C_outflow);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),
                   graphics={Text(
          extent={{-60,60},{60,-60}},
          lineColor={0,128,255},
          textString="ACH")}),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics));
end AirLeakage;
