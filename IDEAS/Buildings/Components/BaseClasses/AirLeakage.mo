within IDEAS.Buildings.Components.BaseClasses;
model AirLeakage "air leakage due to limied air tightness"

extends IDEAS.Fluid.Interfaces.PartialTwoPortInterface(allowFlowReversal=false);

  parameter Modelica.SIunits.Volume V "zone air volume";
  parameter Real n50(min=0.01)=0.4 "n50-value of airtightness";

  parameter SI.Time tau=30 "Tin time constant at nominal flow rate";

  outer IDEAS.SimInfoManager sim "Simulation information manager"
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

  Fluid.Sensors.TemperatureTwoPort senTem(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    tau=tau,
    allowFlowReversal=false)
            annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Fluid.Interfaces.IdealSource       idealSource(
    redeclare package Medium = Medium,
    control_m_flow=true,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{40,60},{60,80}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=sim.Te)
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=V/3600*n50/20)
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  Fluid.MixingVolumes.MixingVolume       vol(
    redeclare package Medium = Medium,
    energyDynamics= Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=m_flow_nominal,
    nPorts=2,
    allowFlowReversal=false,
    final V=1)
    annotation (Placement(transformation(extent={{70,0},{50,20}})));
equation

  connect(port_a, senTem.port_a) annotation (Line(
      points={{-100,0},{-90,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(realExpression.y, prescribedTemperature.T) annotation (Line(
      points={{21,70},{38,70}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(senTem.port_b, idealSource.port_a) annotation (Line(
      points={{-70,0},{0,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(realExpression1.y, idealSource.m_flow_in) annotation (Line(
      points={{-19,30},{4,30},{4,8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(idealSource.port_b, vol.ports[1]) annotation (Line(
      points={{20,0},{62,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.ports[2], port_b) annotation (Line(
      points={{58,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.heatPort, prescribedTemperature.port) annotation (Line(
      points={{70,10},{74,10},{74,70},{60,70}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),
                   graphics={Text(
          extent={{-60,60},{60,-60}},
          lineColor={0,128,255},
          textString="ACH")}),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics));
end AirLeakage;
