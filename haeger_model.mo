model haeger_model
  HumanLib.Valves.Valve_Switching valve_Switching1(R = 0.001)  annotation(
    Placement(visible = true, transformation(origin = {38, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Valves.Valve_Switching valve_Switching2(R = 0.001)  annotation(
    Placement(visible = true, transformation(origin = {-32, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Vessels.Resistance resistance3(R = 1.3)  annotation(
    Placement(visible = true, transformation(origin = {-12, -12}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  HumanLib.Vessels.Compliance C_a(C = 1.6, V_0 = 0)  annotation(
    Placement(visible = true, transformation(origin = {74, -22}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  HumanLib.Vessels.Compliance C_v(C = 1.9, V_0 = 0)  annotation(
    Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Heart.TimeVaryingElastance.Norm_Time norm_Time1 annotation(
    Placement(visible = true, transformation(origin = {-82, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Heart.TimeVaryingElastance.ElastanceFunctions.Elastance_haeger elastance_haeger1(Emax = 1.5, Emin = 0.037, Vd = 1.123)  annotation(
    Placement(visible = true, transformation(origin = {-22, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Heart.Ventricle_Elastance_Simple_Init ventricle_Elastance_Simple_Init1 annotation(
    Placement(visible = true, transformation(origin = {4, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Heart.TimeVaryingElastance.ActivationFunctions.Activation_Winkler activation_Winkler1(Tir = 0.5, Tsys = 0.3)  annotation(
    Placement(visible = true, transformation(origin = {-54, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Vessels.Resistance_init R_v(R = 0.09, p_init = 6)  annotation(
    Placement(visible = true, transformation(origin = {-62, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  HumanLib.Vessels.Resistance_init R_a(R = 0.1, p_init = 96)  annotation(
    Placement(visible = true, transformation(origin = {66, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(C_a.cnBloodStream, resistance3.cnStreamIn) annotation(
    Line(points = {{74, -12}, {-2, -12}}, color = {255, 0, 0}));
  connect(R_a.cnStreamOut, C_a.cnBloodStream) annotation(
    Line(points = {{76, 44}, {76, 22}, {74, 22}, {74, -12}}, color = {255, 0, 0}));
  connect(C_v.cnBloodStream, R_v.cnStreamIn) annotation(
    Line(points = {{-70, 0}, {-70, 22}, {-72, 22}, {-72, 44}}, color = {255, 0, 0}));
  connect(resistance3.cnStreamOut, C_v.cnBloodStream) annotation(
    Line(points = {{-22, -12}, {-70, -12}, {-70, 0}}, color = {255, 0, 0}));
  connect(R_v.cnStreamOut, valve_Switching2.cnStreamIn) annotation(
    Line(points = {{-52, 44}, {-42, 44}}, color = {255, 0, 0}));
  connect(valve_Switching2.cnStreamOut, ventricle_Elastance_Simple_Init1.cnStreamIn) annotation(
    Line(points = {{-22, 44}, {-6, 44}}, color = {255, 0, 0}));
  connect(ventricle_Elastance_Simple_Init1.cnStreamOut, valve_Switching1.cnStreamIn) annotation(
    Line(points = {{14, 44}, {28, 44}}, color = {255, 0, 0}));
  connect(elastance_haeger1.cnPV, ventricle_Elastance_Simple_Init1.cnPV) annotation(
    Line(points = {{-10, 70}, {4, 70}, {4, 51}}, color = {95, 95, 95}));
  connect(valve_Switching1.cnStreamOut, R_a.cnStreamIn) annotation(
    Line(points = {{48, 44}, {56, 44}, {56, 44}, {56, 44}}, color = {255, 0, 0}));
  connect(activation_Winkler1.phi, elastance_haeger1.phi) annotation(
    Line(points = {{-42, 70}, {-36, 70}, {-36, 70}, {-34, 70}}, color = {0, 0, 127}));
  connect(norm_Time1.y, activation_Winkler1.time_norm) annotation(
    Line(points = {{-70, 70}, {-66, 70}}, color = {0, 0, 127}));
end haeger_model;