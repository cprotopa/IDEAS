within IDEAS.Buildings.Components;
model SlabOnGround "opaque floor on ground slab"

extends IDEAS.Buildings.Components.Interfaces.StateWall;

replaceable Data.Interfaces.Construction constructionType(insulationType=insulationType, insulationTickness=insulationThickness)
    "Type of building construction" annotation (choicesAllMatching = true, Placement(transformation(extent={{-38,72},
            {-34,76}})),Dialog(group="Construction details"));
replaceable Data.Interfaces.Insulation insulationType(d=insulationThickness)
    "Type of thermal insulation" annotation (choicesAllMatching = true, Placement(transformation(extent={{-38,84},
            {-34,88}})),Dialog(group="Construction details"));
parameter Modelica.SIunits.Length insulationThickness
    "Thermal insulation thickness" annotation(Dialog(group="Construction details"));
parameter Modelica.SIunits.Area AWall "Total wall area";
parameter Modelica.SIunits.Area PWall "Total wall perimeter";
parameter Modelica.SIunits.Angle inc
    "Inclination of the wall, i.e. 90� denotes vertical";
parameter Modelica.SIunits.Angle azi
    "Azimuth of the wall, i.e. 0� denotes South";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_emb
    "port for gains by embedded active layers"
  annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));

protected
final parameter IDEAS.Buildings.Data.Materials.Ground ground1(d=0.50);
final parameter IDEAS.Buildings.Data.Materials.Ground ground2(d=0.33);
final parameter IDEAS.Buildings.Data.Materials.Ground ground3(d=0.17);

Modelica.SIunits.HeatFlowRate Qm = U*AWall*(22-9)-Lpi*4*cos(2*3.1415/12*(m-1+alfa))+Lpe*9*cos(2*3.1415/12*(m-1-beta));

Modelica.SIunits.Length B = AWall/(0.5*PWall)
    "Characteristic dimension of the slab on ground";
Modelica.SIunits.Length dt = sum(constructionType.mats.d)+ground1.k*layMul.R
    "Equivalent thickness";
Real U = ground1.k/(0.457*B+dt);
Real alfa = 1.5-12/(2*3.14)*atan(dt/(dt+delta));
Real beta = 1.5-0.42*log(delta/(dt+1));
Real delta = sqrt(3.15*10^7*ground1.k/3.14/ground1.rho/ground1.c);
Real Lpi = AWall*ground1.k/dt*sqrt(1/((1+delta/dt)^2+1));
Real Lpe = 0.37*PWall*ground1.k*log(delta/dt+1);
Real m = 12*time/31536000;

protected
  IDEAS.Buildings.Components.BaseClasses.MultiLayerOpaque layMul(
    A=AWall,
    inc=inc,
    nLay=constructionType.nLay,
    mats=constructionType.mats)
    "Declaration of array of resistances and capacitances for wall simulation"
    annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
  IDEAS.Buildings.Components.BaseClasses.InteriorConvection intCon(A=AWall, inc=inc)
    "Convective surface heat transimission on the interior side of the wall"
    annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
  IDEAS.Buildings.Components.BaseClasses.MultiLayerOpaque layGro(
    A=AWall,
    inc=inc,
    nLay=3,
    mats={ground1,ground2,ground3})
    "Declaration of array of resistances and capacitances for ground simulation"
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));

public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow periodicFlow(T_ref=
        284.15)
    annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-30,-8})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature TGro
    annotation (Placement(transformation(extent={{-70,-40},{-50,-20}})));
equation
periodicFlow.Q_flow = Qm;
TGro.T = sim.Tground;

  connect(layMul.port_b, intCon.port_a)       annotation (Line(
      points={{10,-30},{20,-30}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(layGro.port_b, layMul.port_a)             annotation (Line(
      points={{-20,-30},{-10,-30}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(TGro.port, layGro.port_a)                  annotation (Line(
      points={{-50,-30},{-40,-30}},
      color={191,0,0},
      smooth=Smooth.None));

  connect(intCon.port_b, surfCon_a) annotation (Line(
      points={{40,-30},{50,-30}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(layMul.port_b, surfRad_a) annotation (Line(
      points={{10,-30},{16,-30},{16,-60},{50,-60}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(layMul.iEpsLw_b, iEpsLw_a) annotation (Line(
      points={{10,-22},{14,-22},{14,30},{56,30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(layMul.iEpsSw_b, iEpsSw_a) annotation (Line(
      points={{10,-26},{16,-26},{16,0},{56,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(layMul.port_gain[constructionType.locGain], port_emb) annotation (Line(
      points={{0,-40},{0,-100}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(layMul.port_a, periodicFlow.port) annotation (Line(
      points={{-10,-30},{-14,-30},{-14,-8},{-20,-8}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(layMul.area, area_a) annotation (Line(
      points={{0,-20},{0,60},{56,60}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-50,-100},
            {50,100}}),
                   graphics={
        Rectangle(
          extent={{-50,-90},{50,-70}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-50,-20},{-30,-20},{-30,-70},{-30,-70},{-30,-70}},
          color={175,175,175},
          smooth=Smooth.None),
        Line(
          points={{-50,-20},{-50,-90},{-50,-90}},
          color={175,175,175},
          smooth=Smooth.None),
        Line(
          points={{-50,60},{-30,60},{-30,80},{50,80}},
          color={175,175,175},
          smooth=Smooth.None),
        Line(
          points={{-50,60},{-50,66},{-50,100},{50,100}},
          color={175,175,175},
          smooth=Smooth.None),
        Line(
          points={{-44,60},{-44,-20}},
          color={175,175,175},
          smooth=Smooth.None),
        Line(
          points={{-50,-70},{50,-70}},
          color={0,0,0},
          thickness=0.5,
          smooth=Smooth.None)}), Diagram(coordinateSystem(preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}),
                                         graphics),
    Documentation(info="<html>
<p>The <code>SlabOnGround</code> model describes the transient behaviour of a builiding envelope constructions separating a thermal zone with ground massive. The description of the thermal response of a wall is structured as in the 3 different occurring processes, i.e. the heat balance of the outer surface, heat conduction between both surfaces and the heat balance of the interior surface.</p>
<p><h4><font color=\"#008000\">Outer surface heat balance </font></h4></p>
<p>The heat balance of the outer surface in contact to the ground is approximated based on <a href=\"IDEAS.Buildings.UsersGuide.References\">[ISO 13370]</a> based on a steady-state and periodic coupling coefficient. </p>
<p><h4><font color=\"#008000\">Wall conduction process </font></h4></p>
<p>For the purpose of dynamic building simulation, the partial differential equation of the continuous time and space model of heat transport through a solid is most often simplified into ordinary differential equations with a finite number of parameters representing only one-dimensional heat transport through a construction layer. Within this context, the wall is modeled with lumped elements, i.e. a model where temperatures and heat fluxes are determined from a system composed of a sequence of discrete resistances and capacitances R_{n+1}, C_{n}. The number of capacitive elements $n$ used in modeling the transient thermal response of the wall denotes the order of the lumped capacitance model.</p>
<p align=\"center\"><img src=\"modelica://IDEAS/Images/equations/equation-pqp0E04K.png\"/></p>
<p>where <img src=\"modelica://IDEAS/Images/equations/equation-I7KXJhSH.png\"/> is the added energy to the lumped capacity, <img src=\"modelica://IDEAS/Images/equations/equation-B0HPmGTu.png\"/> is the temperature of the lumped capacity, <img src=\"modelica://IDEAS/Images/equations/equation-t7aqbnLB.png\"/> is the thermal capacity of the lumped capacity equal to<img src=\"modelica://IDEAS/Images/equations/equation-JieDs0oi.png\"/> for which rho denotes the density and <img src=\"modelica://IDEAS/Images/equations/equation-ml5CM4zK.png\"/> is the specific heat capacity of the material and <img src=\"modelica://IDEAS/Images/equations/equation-hOGNA6h5.png\"/> the equivalent thickness of the lumped element, where <img src=\"modelica://IDEAS/Images/equations/equation-1pDREAb7.png\"/> the heat flux through the lumped resistance and <img src=\"modelica://IDEAS/Images/equations/equation-XYf3O3hw.png\"/> is the total thermal resistance of the lumped resistance and where <img src=\"modelica://IDEAS/Images/equations/equation-dgS5sGAN.png\"/> are internal thermal source.</p>
<p><h4><font color=\"#008000\">Interior surface heat balance </font></h4></p>
<p>The heat balance of the interior surface is determined as </p>
<p align=\"center\"><img src=\"modelica://IDEAS/Images/equations/equation-omuYJaBt.png\"/></p>
<p>where <img src=\"modelica://IDEAS/Images/equations/equation-VQ2Uq5yQ.png\"/> denotes the heat flow into the wall, <img src=\"modelica://IDEAS/Images/equations/equation-jFQh4YxC.png\"/> denotes heat transfer by convection, <img src=\"modelica://IDEAS/Images/equations/equation-0lIKQUO2.png\"/> denotes short-wave absorption of direct and diffuse solar light netering the interior zone through windows and <img src=\"modelica://IDEAS/Images/equations/equation-jJJ8CeIr.png\"/> denotes long-wave heat exchange with the surounding interior surfaces. The surface heat resistances <img src=\"modelica://IDEAS/Images/equations/equation-mp9YB9Y0.png\"/> for the exterior and interior surface respectively are determined as <img src=\"modelica://IDEAS/Images/equations/equation-heVSJm8U.png\"/> where <img src=\"modelica://IDEAS/Images/equations/equation-81IuiWJg.png\"/> is the surface area and where <img src=\"modelica://IDEAS/Images/equations/equation-HcrpB4Mx.png\"/> is the exterior and interior convective heat transfer coefficient. The interior natural convective heat transfer coefficient <img src=\"modelica://IDEAS/Images/equations/equation-eZGZlJrg.png\"/> is computed for each interior surface as </p>
<p align=\"center\"><img src=\"modelica://IDEAS/Images/equations/equation-KNBSKUDK.png\"/></p>
<p>where <img src=\"modelica://IDEAS/Images/equations/equation-W5kvS3SS.png\"/> is the characteristic length of the surface, <img src=\"modelica://IDEAS/Images/equations/equation-jhC1rqax.png\"/> is the indoor air temperature and <img src=\"modelica://IDEAS/Images/equations/equation-sbXAgHuQ.png\"/> are correlation coefficients. These parameters {<img src=\"modelica://IDEAS/Images/equations/equation-nHmmePq5.png\"/>,<img src=\"modelica://IDEAS/Images/equations/equation-zJZmNUzp.png\"/>,<img src=\"modelica://IDEAS/Images/equations/equation-7nwXbcLp.png\"/>} are identical to {1.823,-0.121,0.293} for vertical surfaces <a href=\"IDEAS.Buildings.UsersGuide.References\">[Khalifa 2001]</a>, {2.175,-0.076,0.308} for horizontal surfaces wherefore the heat flux is in the same direction as the buoyancy force <a href=\"IDEAS.Buildings.UsersGuide.References\">[Khalifa 2001]</a>, and {2.72,-,0.13} for horizontal surfaces wherefore the heat flux is in the opposite direction as the buoyancy force <a href=\"IDEAS.Buildings.UsersGuide.References\">[Awbi 1999]</a>. The interior natural convective heat transfer coefficient is only described as function of the temperature difference. Similar to the thermal model for heat transfer through a wall, a thermal circuit formulation for the direct radiant exchange between surfaces can be derived <a href=\"IDEAS.Buildings.UsersGuide.References\">[Buchberg 1955, Oppenheim 1956]</a>. The resulting heat exchange by longwave radiation between two surface <img src=\"modelica://IDEAS/Images/equations/equation-u44Z52zq.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-1i2Mx4Ca.png\"/> can be described as</p>
<p align=\"center\"><img src=\"modelica://IDEAS/Images/equations/equation-ULwzvhTg.png\"/></p>
<p>as derived from the Stefan-Boltzmann law wherefore <img src=\"modelica://IDEAS/Images/equations/equation-uDITjyTv.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-zRzZUsty.png\"/> are the emissivity of surfaces <img src=\"modelica://IDEAS/Images/equations/equation-AtoPkohW.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-XBM6Gmdj.png\"/> respectively, <img src=\"modelica://IDEAS/Images/equations/equation-l77CVDpb.png\"/> is radiant-interchange configuration factor <a href=\"IDEAS.Buildings.UsersGuide.References\">[Hamilton 1952]</a> between surfaces <img src=\"modelica://IDEAS/Images/equations/equation-40yn0CNI.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-8AtnOwyb.png\"/>, <img src=\"modelica://IDEAS/Images/equations/equation-RBEWjWKa.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-fxxW5qJd.png\"/> are the areas of surfaces <img src=\"modelica://IDEAS/Images/equations/equation-OM5mvW83.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-2vsckTJX.png\"/> respectively, sigma is the Stefan-Boltzmann constant <a href=\"IDEAS.Buildings.UsersGuide.References\">[Mohr 2008]</a> and <img src=\"modelica://IDEAS/Images/equations/equation-I8IpXvtl.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-HjTUUaGT.png\"/> are the surface temperature of surfaces <img src=\"modelica://IDEAS/Images/equations/equation-pkFYc6TO.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-sbLFxSp8.png\"/> respectively. The above description of longwave radiation as mentioned above for a room or thermal zone results in the necessity of a very detailed input, i.e. the configuration between all needs to be described by their shape, position and orientation in order to define <img src=\"modelica://IDEAS/Images/equations/equation-MRIBXe5x.png\"/>, and difficulties to introduce windows and internal gains in the zone of interest. Simplification is achieved by means of a <img src=\"modelica://IDEAS/Images/equations/equation-tEXNH19s.png\"/> or delta-star transformation <a href=\"IDEAS.Buildings.UsersGuide.References\">[Kenelly 1899]</a> and by definition of a (fictive) radiant star node in the zone model. Literature <a href=\"IDEAS.Buildings.UsersGuide.References\">[Liesen 1997]</a> shows that the overall model is not significantly sensitive to this assumption. The heat exchange by longwave radiation between surface <img src=\"modelica://IDEAS/Images/equations/equation-Mjd7rCtc.png\"/> and the radiant star node in the zone model can be described as</p>
<p align=\"center\"><img src=\"modelica://IDEAS/Images/equations/equation-DsCmBlzm.png\"/></p>
<p>where <img src=\"modelica://IDEAS/Images/equations/equation-kKTzmHby.png\"/> is the emissivity of surface <img src=\"modelica://IDEAS/Images/equations/equation-QxeNUrgq.png\"/>, <img src=\"modelica://IDEAS/Images/equations/equation-ERb44Mb3.png\"/> is the area of surface <img src=\"modelica://IDEAS/Images/equations/equation-XF12EapA.png\"/>, <img src=\"modelica://IDEAS/Images/equations/equation-yyuov0Oi.png\"/> is the sum of areas for all surfaces <img src=\"modelica://IDEAS/Images/equations/equation-yRwsej2e.png\"/> of the thermal zone, <img src=\"modelica://IDEAS/Images/equations/equation-BmOQFx4B.png\"/> is the Stefan-Boltzmann constant <a href=\"IDEAS.Buildings.UsersGuide.References\">[Mohr 2008]</a> and <img src=\"modelica://IDEAS/Images/equations/equation-kDraFb4h.png\"/> and <img src=\"modelica://IDEAS/Images/equations/equation-suo6xr9a.png\"/> are the temperatures of surfaces <img src=\"modelica://IDEAS/Images/equations/equation-olgnuMEg.png\"/> and the radiant star node respectively. Absorption of shortwave solar radiation on the interior surface is handled equally as for the outside surface. Determination of the receiving solar radiation on the interior surface after passing through windows is dealt with in the zone model.</p>
</html>"));
end SlabOnGround;