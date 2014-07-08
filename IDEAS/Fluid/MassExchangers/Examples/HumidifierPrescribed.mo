within IDEAS.Fluid.MassExchangers.Examples;
model HumidifierPrescribed "Model that demonstrates the ideal humidifier model"
  extends Modelica.Icons.Example;

  package Medium = IDEAS.Media.Air;

  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal=
     3000/1000/20 "Nominal mass flow rate";

  IDEAS.Fluid.Sources.MassFlowSource_T sou(
    redeclare package Medium = Medium,
    use_T_in=false,
    nPorts=2,
    m_flow=2*m_flow_nominal,
    T=303.15) "Source" annotation (Placement(transformation(extent={{-82,40},{-62,
            60}}, rotation=0)));
  IDEAS.Fluid.MassExchangers.HumidifierPrescribed humSte(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=6000,
    tau=0,
    mWat_flow_nominal=m_flow_nominal*0.005,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    "Steady-state model of the humidifier"
    annotation (Placement(transformation(extent={{0,90},{20,110}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem1(redeclare package Medium
      = Medium, m_flow_nominal=m_flow_nominal) "Temperature sensor"
    annotation (Placement(transformation(extent={{40,90},{60,110}})));
  Modelica.Blocks.Sources.TimeTable TSet(table=[0, 273.15 + 30; 120, 273.15 +
        30; 120, 273.15 + 25; 1200, 273.15 + 25]) "Setpoint"
    annotation (Placement(transformation(extent={{-60,140},{-40,160}})));
  IDEAS.Controls.Continuous.LimPID con1(
    Td=1,
    k=1,
    Ti=10,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    reverseAction=true) "Controller"
    annotation (Placement(transformation(extent={{40,140},{60,160}})));
  IDEAS.Fluid.MassExchangers.HumidifierPrescribed humDyn(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=6000,
    mWat_flow_nominal=m_flow_nominal*0.005,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    T_start=303.15) "Dynamic model of the humidifier"
    annotation (Placement(transformation(extent={{0,-20},{20,0}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem2(redeclare package Medium
      = Medium, m_flow_nominal=m_flow_nominal) "Temperature sensor"
    annotation (Placement(transformation(extent={{40,-20},{60,0}})));
  IDEAS.Controls.Continuous.LimPID con2(
    Td=1,
    Ti=10,
    k=0.1,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    reverseAction=true) "Controller"
    annotation (Placement(transformation(extent={{40,30},{60,50}})));
  IDEAS.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    use_T_in=false,
    p(displayUnit="Pa"),
    T=303.15,
    nPorts=2) "Sink"   annotation (Placement(transformation(extent={{178,40},{158,
            60}}, rotation=0)));
equation
  connect(senTem1.T, con1.u_m) annotation (Line(
      points={{50,111},{50,138}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TSet.y, con1.u_s) annotation (Line(
      points={{-39,150},{38,150}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(con1.y, humSte.u) annotation (Line(
      points={{61,150},{70,150},{70,130},{-10,130},{-10,106},{-2,106}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(humSte.port_b, senTem1.port_a) annotation (Line(
      points={{20,100},{40,100}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem2.T, con2.u_m) annotation (Line(
      points={{50,1},{50,28}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TSet.y, con2.u_s) annotation (Line(
      points={{-39,150},{-14,150},{-14,40},{38,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(con2.y, humDyn.u) annotation (Line(
      points={{61,40},{70,40},{70,20},{-10,20},{-10,-4},{-2,-4}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(humDyn.port_b, senTem2.port_a) annotation (Line(
      points={{20,-10},{40,-10}},
      color={0,127,255},
      smooth=Smooth.None));

  connect(sou.ports[1], humSte.port_a) annotation (Line(
      points={{-62,52},{-30,52},{-30,100},{0,100}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sou.ports[2], humDyn.port_a) annotation (Line(
      points={{-62,48},{-30,48},{-30,-10},{-4.44089e-16,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem1.port_b, sin.ports[1]) annotation (Line(
      points={{60,100},{109,100},{109,52},{158,52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem2.port_b, sin.ports[2]) annotation (Line(
      points={{60,-10},{108,-10},{108,48},{158,48}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{200,
            200}}), graphics),
    __Dymola_Commands(file=
          "modelica://IDEAS/Resources/Scripts/Dymola/Fluid/MassExchangers/Examples/HumidifierPrescribed.mos"
        "Simulate and plot"),
    Documentation(info="<html>
<p>
Model that demonstrates the use of an ideal humidifier.
Both humidifer models are identical, except that one model is configured
as a steady-state model, whereas the other is configured as a dynamic model.
Both humidifiers add water to the medium to track a set-point for the outlet
temperature using adiabatic cooling.
The temperature of the water that is added to the medium is determined by
the parameter <code>T</code> of the humidifier models.
</p>
</html>", revisions="<html>
<ul>
<li>
April 23, 2013, by Michael Wetter:<br/>
Added flow resistance at the outlet of the humidifier to avoid a numerical derivative,
and changed model to use a prescribed mass flow rate.
</li>
<li>
July 11, 2011, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    experiment(
      StopTime=1200,
      Tolerance=1e-05));
end HumidifierPrescribed;
