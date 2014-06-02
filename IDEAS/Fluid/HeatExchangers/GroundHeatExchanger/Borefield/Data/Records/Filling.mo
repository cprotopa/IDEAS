within IDEAS.Fluid.HeatExchangers.GroundHeatExchanger.Borefield.Data.Records;
record Filling "Thermal properties of the filling material of the boreholes"
  extends IDEAS.HeatTransfer.Data.BoreholeFillings.Generic;

  parameter String name="Filling";
  final parameter Modelica.SIunits.DiffusionCoefficient alp=k/d/c;

end Filling;
