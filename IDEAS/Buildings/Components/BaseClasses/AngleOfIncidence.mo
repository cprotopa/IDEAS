within IDEAS.Buildings.Components.BaseClasses;
model AngleOfIncidence "angle of incidence"

  Modelica.Blocks.Interfaces.RealInput angInc "angle of incidence in radians"
    annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
  Modelica.Blocks.Interfaces.RealOutput angIncDeg
    "angle of incidence in degrees for lookup table"
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));

algorithm
  angIncDeg := Modelica.SIunits.Conversions.to_deg(angInc);

end AngleOfIncidence;
