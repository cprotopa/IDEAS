within IDEAS.Fluid.HeatExchangers.GroundHeatExchangers.Borefield.Examples;
model ercan
  "Model of a borefield in a 8x1 boreholes line configuration and a constant heat injection rate. The descritization is 600 second"

  extends Modelica.Icons.Example;

  package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

  parameter Data.BorefieldData.SandStone_Bentonite_c8x1_h110_b5_d600_T283
    bfData
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  parameter Integer lenSim=3600*24*365*10 "length of the simulation";

  MultipleBoreHoles multipleBoreholes(lenSim=lenSim, bfData=bfData,
    redeclare package Medium = Medium) "borefield"
    annotation (Placement(transformation(extent={{-20,-60},{20,-20}})));
  Modelica.Blocks.Sources.Step           load(height=1, startTime=36000)
    "load for the borefield"
    annotation (Placement(transformation(extent={{20,-20},{34,-6}})));

  Movers.Pump                           pum(
    redeclare package Medium = Medium,
    useInput=true,
    T_start=bfData.gen.T_start,
    m_flow(start=bfData.m_flow_nominal),
    m_flow_nominal=bfData.m_flow_nominal)
    annotation (Placement(transformation(extent={{-10,22},{-30,2}})));
  Modelica.Blocks.Sources.Constant mFlo(k=1)
    annotation (Placement(transformation(extent={{-46,-12},{-34,0}})));
  HeaterCoolerPrescribed                            hea(
    redeclare package Medium = Medium,
    dp_nominal=10000,
    show_T=true,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    T_start=bfData.gen.T_start,
    m_flow_nominal=bfData.m_flow_nominal,
    m_flow(start=bfData.m_flow_nominal),
    Q_flow_nominal=bfData.gen.q_ste*bfData.gen.nbBh*bfData.gen.hBor,
    p_start=100000)
    annotation (Placement(transformation(extent={{30,22},{10,2}})));
  Modelica.Fluid.Sources.Boundary_pT boundary(nPorts=1, redeclare package
      Medium = Medium)
    annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  Sensors.TemperatureTwoPort senTem(
    redeclare package Medium = Medium,
    m_flow_nominal=bfData.m_flow_nominal,
    T_start=bfData.gen.T_start)
    annotation (Placement(transformation(extent={{38,-50},{58,-30}})));
  Sensors.TemperatureTwoPort senTem1(
    redeclare package Medium = Medium,
    m_flow_nominal=bfData.m_flow_nominal,
    T_start=bfData.gen.T_start)
    annotation (Placement(transformation(extent={{-52,-50},{-32,-30}})));
  Modelica.Blocks.Interfaces.RealOutput THcf_out
    "Temperature of the heat carrier fluid at the outlet of the borefield"
    annotation (Placement(transformation(extent={{90,80},{110,100}})));
  Modelica.Blocks.Interfaces.RealOutput THcf_in
    "Temperature of the heat carrier fluid at the inlet of the borefield"
    annotation (Placement(transformation(extent={{90,20},{110,40}})));
  Modelica.Blocks.Interfaces.RealOutput THcf_ave
    "average temperature of the heat carrier fluid between in- and outlet of the borefield"
    annotation (Placement(transformation(extent={{90,-40},{110,-20}})));
  Modelica.Blocks.Interfaces.RealOutput QLoa "Total load to the borefield"
    annotation (Placement(transformation(extent={{90,-90},{110,-70}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=senTem.T)
    annotation (Placement(transformation(extent={{46,80},{66,100}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=senTem1.T)
    annotation (Placement(transformation(extent={{46,20},{66,40}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=(senTem1.T + senTem.T)
        /2) annotation (Placement(transformation(extent={{50,-82},{70,-62}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=hea.Q_flow_nominal*
        hea.u)
    annotation (Placement(transformation(extent={{50,-104},{70,-84}})));
equation
  connect(pum.port_a,hea. port_b) annotation (Line(
      points={{-10,12},{10,12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(mFlo.y,pum. m_flowSet) annotation (Line(
      points={{-33.4,-6},{-20,-6},{-20,1.6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(load.y, hea.u) annotation (Line(
      points={{34.7,-13},{52,-13},{52,6},{32,6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(boundary.ports[1], pum.port_a) annotation (Line(
      points={{-40,50},{-10,50},{-10,12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hea.port_a, senTem.port_b) annotation (Line(
      points={{30,12},{70,12},{70,-40},{58,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem.port_a, multipleBoreholes.port_b) annotation (Line(
      points={{38,-40},{20,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pum.port_b, senTem1.port_a) annotation (Line(
      points={{-30,12},{-60,12},{-60,-40},{-52,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem1.port_b, multipleBoreholes.port_a) annotation (Line(
      points={{-32,-40},{-20,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(realExpression.y, THcf_out) annotation (Line(
      points={{67,90},{100,90}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression2.y, THcf_ave) annotation (Line(
      points={{71,-72},{76,-72},{76,-30},{100,-30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression3.y, QLoa) annotation (Line(
      points={{71,-94},{80,-94},{80,-80},{100,-80}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(realExpression1.y, THcf_in) annotation (Line(
      points={{67,30},{81.5,30},{81.5,30},{100,30}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}}), graphics),
    experiment(StopTime=1.7e+006, __Dymola_NumberOfIntervals=100),
    __Dymola_experimentSetupOutput);
end ercan;
