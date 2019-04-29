package HumanLib
  package Basics
    package Units
      type Pressure_mmHg = Real(final quantity = "Pressure", final unit = "mmHg", final displayUnit = "mmHg") "1 mmHg = 1 Torr = 133.322 Pa";
      //type Pressure_mmHg = Real (final quantity="Pressure")
      //  "1 mmHg = 1 Torr = 133.322 Pa";
      type Volume_ml = Real(final quantity = "Volume", final unit = "ml");
      type VolumeFlowRate_mls = Real(final quantity = "VolumeFlowRate", final unit = "ml/s");
      type VolumeFlowRate_lmin = Real(final quantity = "VolumeFlowRate", final unit = "l/min");
      type VolumeFlowRate_ls = Real(final quantity = "VolumeFlowRate", final unit = "l/s");
      type HydraulicResistance = Real(final quantity = "HydraulicResistance", final unit = "mmHg.s/ml");
      //type Compliancy = Real (final quantity="Compliancy", final unit="ml/mmHg");
      type Compliancy = Real(final quantity = "Compliancy", final unit = "ml/mmHg");
      type HydraulicInertia = Real(final quantity = "HydraulicInertia", final unit = "mmHg.s2/ml");
      type Elastance = Real(final quantity = "Elastance", final unit = "mmHg/ml");
    end Units;

    package Interfaces
      connector cnBloodStream
        extends Icons.icoStream;
        HumanLib.Basics.Units.Pressure_mmHg P;
        flow HumanLib.Basics.Units.VolumeFlowRate_mls VolFlow;
      end cnBloodStream;

      connector cnBloodStreamIn
        extends Interfaces.cnBloodStream;
        extends Icons.icoStreamIn;
      end cnBloodStreamIn;

      connector cnBloodStreamOut
        extends Interfaces.cnBloodStream;
        extends Icons.icoStreamOut;
      end cnBloodStreamOut;

      partial model BloodStreamTwoPin "Ein-Ausgangsumgebung fuer alle Modelle, die Stream-connectors benutzen"
        cnBloodStreamIn cnStreamIn annotation(
          Placement(transformation(extent = {{-110, -10}, {-90, 10}}, rotation = 0)));
        cnBloodStreamOut cnStreamOut annotation(
          Placement(transformation(extent = {{90, -10}, {110, 10}}, rotation = 0)));
        Basics.Units.Pressure_mmHg dP;
      equation
        dP = cnStreamIn.P - cnStreamOut.P;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics));
      end BloodStreamTwoPin;

      partial model BloodStreamOnePort
        extends BloodStreamTwoPin;
        Basics.Units.VolumeFlowRate_mls VolFlow;
      equation
        0 = cnStreamIn.VolFlow + cnStreamOut.VolFlow;
        VolFlow = cnStreamIn.VolFlow;
      end BloodStreamOnePort;

      connector cnPV
        HumanLib.Basics.Units.Pressure_mmHg P;
        HumanLib.Basics.Units.Volume_ml V;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-40, -40}, {40, 40}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, lineThickness = 0.5, pattern = LinePattern.None, lineColor = {95, 95, 95})}),
          Diagram(coordinateSystem(extent = {{-40, -40}, {40, 40}})));
      end cnPV;

      connector cnPVOut
        output HumanLib.Basics.Units.Pressure_mmHg P;
        input HumanLib.Basics.Units.Volume_ml V;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {28, 108, 200}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid)}));
      end cnPVOut;

      connector cnPVIn
        input HumanLib.Basics.Units.Pressure_mmHg P;
        output HumanLib.Basics.Units.Volume_ml V;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {28, 108, 200}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid)}));
      end cnPVIn;
    end Interfaces;

    package Icons
      connector icoStream
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {255, 0, 0}, lineThickness = 0.5), Ellipse(extent = {{-92, 92}, {92, -92}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.Solid, fillColor = {255, 0, 0})}));
      end icoStream;

      connector icoStreamIn
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {255, 0, 0}, lineThickness = 0.5), Ellipse(extent = {{-92, 92}, {92, -92}}, lineColor = {0, 0, 0}, lineThickness = 1, fillPattern = FillPattern.Sphere, fillColor = {255, 0, 0})}));
      end icoStreamIn;

      connector icoStreamOut
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {255, 0, 0}, lineThickness = 0.5), Ellipse(extent = {{-92, 92}, {92, -92}}, lineColor = {0, 0, 0}, lineThickness = 1, fillPattern = FillPattern.Sphere, fillColor = {255, 0, 0}), Ellipse(extent = {{-52, 52}, {52, -52}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.Sphere, fillColor = {139, 0, 0})}));
      end icoStreamOut;

      partial model icoValve
        extends icoNameBottom;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-20, 50}, {-20, -50}, {20, 0}, {18, 2}, {-20, 50}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{-90, 20}, {-20, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{20, 20}, {90, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0})}));
      end icoValve;

      partial model icoCompliance_OnePin
        extends icoNameBottom;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{-46, 44}, {46, -44}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{-40, 20}, {-90, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0})}));
      end icoCompliance_OnePin;

      partial model icoCompliance_TwoPin
        extends icoCompliance_OnePin;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{90, 20}, {40, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0})}));
      end icoCompliance_TwoPin;

      partial model icoInertance
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{-90, 20}, {-40, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{40, 20}, {90, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0})}));
      end icoInertance;

      partial model icoHydraulicResistor
        extends icoNameBottom;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-40, 40}, {-40, -40}, {0, 0}, {-40, 40}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid), Polygon(points = {{0, 0}, {40, 40}, {40, -40}, {0, 0}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-90, 20}, {-40, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{40, 20}, {90, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0})}));
      end icoHydraulicResistor;

      partial model icoVAD
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Ellipse(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 1), Line(points = {{98, -20}, {-60, -80}}, color = {0, 0, 0}, thickness = 1), Line(points = {{98, 20}, {-60, 80}}, color = {0, 0, 0}, thickness = 1), Line(points = {{-80, 0}, {60, 0}, {40, 20}}, color = {238, 46, 47}, thickness = 0.5), Line(points = {{60, 0}, {40, -20}}, color = {238, 46, 47}, thickness = 0.5)}));
      end icoVAD;

      partial model icoNameBottom
        annotation(
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          Icon(graphics = {Text(extent = {{-100, -80}, {100, -100}}, lineColor = {28, 108, 200}, lineThickness = 0.5, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, textString = "%name")}));
      end icoNameBottom;
    end Icons;

    package Sources
      partial model PressureSource_Partial
        Interfaces.cnBloodStream cnBloodStream annotation(
          Placement(transformation(extent = {{70, -70}, {90, -50}}), iconTransformation(extent = {{70, -70}, {90, -50}})));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -100}, {100, 100}}), graphics = {Line(points = {{80, 64}, {80, -76}, {-60, -76}, {-60, 64}}, color = {0, 0, 0}, thickness = 0.5), Rectangle(extent = {{-60, 30}, {80, -76}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid), Polygon(points = {{22, 50}, {40, 30}, {58, 50}, {22, 50}}, lineColor = {0, 0, 0}, lineThickness = 0.5)}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -100}, {100, 100}})));
      end PressureSource_Partial;

      model PressureSource_Variable "Supplies blood pressure through real input (corresponds to infinite volume resevoir)"
        extends PressureSource_Partial;
        input Modelica.Blocks.Interfaces.RealInput P annotation(
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-120, 50}), iconTransformation(extent = {{-100, 10}, {-60, 50}})));
      equation
        cnBloodStream.P = P;
        annotation(
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -100}, {100, 100}})));
      end PressureSource_Variable;

      model ConstantPressureBoundary
        extends PressureSource_Partial;
        parameter Basics.Units.Pressure_mmHg p(start = 0);
      equation
        cnBloodStream.P = p;
      end ConstantPressureBoundary;

      model TestTrajectory
        parameter Real A, B, C, t1, t2, t3;
        Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(transformation(extent = {{96, -10}, {116, 10}})));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-100, 20}, {-64, 20}, {-26, 52}, {36, 52}, {70, -20}, {100, -20}}, color = {28, 108, 200}, thickness = 0.5), Text(extent = {{-92, 38}, {-76, 28}}, lineColor = {28, 108, 200}, textString = "A"), Text(extent = {{-6, 70}, {10, 60}}, lineColor = {28, 108, 200}, textString = "B"), Text(extent = {{76, -4}, {92, -14}}, lineColor = {28, 108, 200}, textString = "C"), Text(extent = {{-72, 16}, {-56, 6}}, lineColor = {28, 108, 200}, textString = "t1"), Text(extent = {{-32, 46}, {-16, 36}}, lineColor = {28, 108, 200}, textString = "t2"), Text(extent = {{24, 46}, {40, 36}}, lineColor = {28, 108, 200}, textString = "t3"), Text(extent = {{62, -22}, {78, -32}}, lineColor = {28, 108, 200}, textString = "t4"), Rectangle(extent = {{-100, 80}, {100, -80}}, lineColor = {0, 0, 0}, lineThickness = 0.5)}));
      end TestTrajectory;

      partial model PVSource_Partial
        Interfaces.cnPV cnPV annotation(
          Placement(transformation(extent = {{-150, 24}, {-104, 74}}), iconTransformation(extent = {{-60, -10}, {-80, 10}})));
        Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(transformation(extent = {{100, -26}, {60, 14}}), iconTransformation(extent = {{88, -14}, {60, 14}})));
      equation

        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-60, -40}, {60, 40}}), graphics = {Rectangle(extent = {{-60, 40}, {60, -40}}, lineColor = {0, 0, 0})}),
          Diagram(coordinateSystem(extent = {{-60, -40}, {60, 40}})));
      end PVSource_Partial;

      model PVSource_P
        extends PVSource_Partial;
      equation
        cnPV.P = u;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-60, 40}, {60, -40}}, lineColor = {95, 95, 95}, lineThickness = 0.5, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, textString = "P")}));
      end PVSource_P;

      model PVSource_V
        extends PVSource_Partial;
      equation
        cnPV.V = u;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-60, 40}, {60, -40}}, lineColor = {95, 95, 95}, lineThickness = 0.5, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, textString = "V")}));
      end PVSource_V;

      partial model FlowSource_Partial
        Interfaces.cnBloodStream cnBloodStream annotation(
          Placement(transformation(extent = {{72, -10}, {92, 10}}), iconTransformation(extent = {{72, -10}, {92, 10}})));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -100}, {100, 100}}), graphics = {Rectangle(extent = {{80, 20}, {-20, -20}}, lineColor = {0, 0, 0}, lineThickness = 0.5), Line(points = {{-60, 0}, {20, 0}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{-60, 40}, {-60, -40}}, color = {0, 0, 0}, thickness = 0.5), Rectangle(extent = {{20, 20}, {80, -20}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, lineThickness = 0.5), Rectangle(extent = {{18, 20}, {20, -20}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -100}, {100, 100}})));
      end FlowSource_Partial;

      model ConstantFlowBoundary
        extends FlowSource_Partial;
        parameter Basics.Units.VolumeFlowRate_mls q(start = 0);
      equation
        cnBloodStream.VolFlow = -q;
      end ConstantFlowBoundary;

      model FlowSource_Variable
        extends FlowSource_Partial;
        Modelica.Blocks.Interfaces.RealInput Q annotation(
          Placement(transformation(extent = {{-100, -20}, {-60, 20}}), iconTransformation(extent = {{-100, -20}, {-60, 20}})));
      equation
        cnBloodStream.VolFlow = -Q;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -100}, {100, 100}})));
      end FlowSource_Variable;
    end Sources;

    package Sensors
      model PressureSensor
        extends Modelica.Icons.RotationalSensor;
        Interfaces.cnBloodStream cnBloodStream annotation(
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Blocks.Interfaces.RealOutput p annotation(
          Placement(transformation(extent = {{96, -10}, {116, 10}})));
      equation
        p = cnBloodStream.P;
        cnBloodStream.VolFlow = 0;
      end PressureSensor;

      model FlowSensor
        extends Modelica.Icons.RotationalSensor;
        extends Basics.Interfaces.BloodStreamOnePort;
        Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(transformation(extent = {{0, -100}, {20, -80}}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -94})));
      equation
        cnStreamIn.P = cnStreamOut.P;
        y = cnStreamIn.VolFlow;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
      end FlowSensor;

      block MovingAverage "Determines the mean value and standard deviation of a signal for a fixed time interval T."
        extends Modelica.Blocks.Interfaces.BlockIcon;
        import SI = Modelica.SIunits;
        parameter SI.Time T = 0.1 "Time interval used for calculating mean value and standard deviation";
        parameter Integer n = 10 "number of increments in T";
        parameter Real y_0 = 0 "Valu to initialise the interval with.";
        Modelica.Blocks.Interfaces.RealInput u "signal input" annotation(
          Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
        Modelica.Blocks.Interfaces.RealOutput y "average value" annotation(
          Placement(transformation(extent = {{100, -10}, {120, 10}})));
      protected
        parameter SI.Time dt = T / n "Precision of monitor";
        Real[n] uArray;
      initial equation
        uArray = ones(n) * y_0;
      equation
        when sample(0, dt) then
          uArray[1] = u;
          for j in 2:n loop
            uArray[j] = pre(uArray[j - 1]);
          end for;
        end when;
        y = sum(uArray) / n;
// mean value
        annotation(
          Diagram(graphics));
      end MovingAverage;

      model PVSensor
        Modelica.Blocks.Interfaces.RealOutput P annotation(
          Placement(transformation(extent = {{40, 30}, {60, 50}}), iconTransformation(extent = {{40, 26}, {68, 54}})));
        Modelica.Blocks.Interfaces.RealOutput V annotation(
          Placement(transformation(extent = {{40, -50}, {60, -30}}), iconTransformation(extent = {{40, -54}, {68, -26}})));
        Interfaces.cnPV cnPV annotation(
          Placement(transformation(extent = {{-94, -26}, {-22, 46}}), iconTransformation(extent = {{-60, -10}, {-40, 10}})));
      equation
        P = cnPV.P;
        V = cnPV.V;
        annotation(
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-40, -80}, {40, 80}})),
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-40, -80}, {40, 80}}), graphics = {Text(extent = {{30, 24}, {-38, 54}}, lineColor = {28, 108, 200}, horizontalAlignment = TextAlignment.Right, textString = "P"), Text(extent = {{30, -56}, {-38, -26}}, lineColor = {28, 108, 200}, horizontalAlignment = TextAlignment.Right, textString = "V"), Rectangle(extent = {{-40, 80}, {40, -80}}, lineColor = {0, 0, 0})}));
      end PVSensor;

      block MeanValue
        extends Modelica.Blocks.Interfaces.SISO;
        Real int_u(start = 0, fixed = true);
      equation
        der(int_u) = u;
        if time > 1e-3 then
          y = int_u / time;
        else
          y = u;
        end if;
      end MeanValue;
    end Sensors;
  end Basics;

  package Vessels
    model Resistance_Partial
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoHydraulicResistor;
      parameter Basics.Units.HydraulicResistance R;
    end Resistance_Partial;

    model Resistance
      extends Resistance_Partial;
    equation
      dP = VolFlow * R;
    end Resistance;

    model Resistance_Turbulent
      extends Resistance_Partial;
      parameter Real R_Turb = 0.1;
    equation
      dP = VolFlow * R + VolFlow ^ 2 * R_Turb;
    end Resistance_Turbulent;

    model Resistance_Variable
      extends Resistance_Partial;
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(transformation(extent = {{20, -20}, {-20, 20}}, rotation = 90, origin = {0, 104})));
    equation
      dP = VolFlow * (u + R);
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{10, 100}, {40, 74}}, lineColor = {28, 108, 200}, textString = "R")}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end Resistance_Variable;

    model Compliance
      extends Basics.Icons.icoCompliance_OnePin;
      parameter Basics.Units.Compliancy C;
      parameter Basics.Units.Volume_ml V_0 = 0;
      //Unstressed volume
      Basics.Units.Volume_ml V;
      Basics.Interfaces.cnBloodStream cnBloodStream annotation(
        Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
    equation
      der(V) = cnBloodStream.VolFlow;
      cnBloodStream.P = 1 / C * (V - V_0);
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end Compliance;

    model Compliance_TwoPin
      extends Basics.Icons.icoCompliance_TwoPin;
      extends Basics.Interfaces.BloodStreamTwoPin;
      parameter Basics.Units.Compliancy C;
      parameter Basics.Units.Volume_ml V_0;
      //Unstressed volume
      Basics.Units.Volume_ml V;
    equation
      cnStreamIn.P = cnStreamOut.P;
      der(V) = cnStreamIn.VolFlow + cnStreamOut.VolFlow;
      cnStreamIn.P = 1 / C * (V - V_0);
    end Compliance_TwoPin;

    model Compliance_PressureDiff
      extends Basics.Icons.icoCompliance_TwoPin;
      extends Basics.Interfaces.BloodStreamTwoPin;
      parameter Basics.Units.Compliancy C;
      parameter Basics.Units.Volume_ml V_0;
      //Unstressed volume
      Basics.Units.Volume_ml V;
    equation
      cnStreamIn.VolFlow + cnStreamOut.VolFlow = 0;
      der(V) = cnStreamIn.VolFlow + cnStreamOut.VolFlow;
      dP = 1 / C * (V - V_0);
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{0, 44}, {0, -44}}, color = {0, 0, 0}, thickness = 0.5)}));
    end Compliance_PressureDiff;

    model Inductance
      extends Basics.Icons.icoInertance;
      extends Basics.Interfaces.BloodStreamOnePort;
      parameter Basics.Units.HydraulicInertia L;
    equation
      dP = L * der(VolFlow);
    end Inductance;

    model Nordergraaf
      extends Basics.Interfaces.BloodStreamTwoPin;
      Inductance inductance(L = L) annotation(
        Placement(transformation(extent = {{-14, 26}, {6, 46}})));
      parameter Basics.Units.HydraulicInertia L;
      Compliance compliance_2(C = C) annotation(
        Placement(transformation(extent = {{20, -40}, {40, -20}})));
      parameter Basics.Units.Compliancy C, C1;
      Compliance compliance_1(C = C1) annotation(
        Placement(transformation(extent = {{-22, -40}, {-2, -20}})));
      Resistance resistance(R = R) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-22, 2})));
      parameter Basics.Units.HydraulicResistance R;
    equation
      connect(cnStreamIn, inductance.cnStreamIn) annotation(
        Line(points = {{-100, 0}, {-54, 0}, {-54, 36}, {-14, 36}}, color = {255, 0, 0}, thickness = 0.5));
      connect(cnStreamOut, inductance.cnStreamOut) annotation(
        Line(points = {{100, 0}, {56, 0}, {56, 36}, {6, 36}}, color = {255, 0, 0}, thickness = 0.5));
      connect(inductance.cnStreamOut, compliance_2.cnBloodStream) annotation(
        Line(points = {{6, 36}, {12, 36}, {12, -30}, {20, -30}}, color = {255, 0, 0}, thickness = 0.5));
      connect(inductance.cnStreamIn, resistance.cnStreamOut) annotation(
        Line(points = {{-14, 36}, {-22, 36}, {-22, 12}}, color = {255, 0, 0}, thickness = 0.5));
      connect(resistance.cnStreamIn, compliance_1.cnBloodStream) annotation(
        Line(points = {{-22, -8}, {-22, -30}}, color = {255, 0, 0}, thickness = 0.5));
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -160}, {100, 40}})),
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -160}, {100, 40}}), graphics = {Rectangle(extent = {{-30, 40}, {30, -40}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{-100, 20}, {-30, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Rectangle(extent = {{30, 20}, {100, -20}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Polygon(points = {{-1.14424e-016, 20}, {20, 40}, {20, 3.36469e-016}, {-1.14424e-016, 20}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, origin = {-40, -68}, rotation = 90), Rectangle(extent = {{-14, 14}, {14, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {-60, -34}, rotation = 90), Polygon(points = {{-1.14424e-016, 20}, {20, 40}, {20, 3.36469e-016}, {-1.14424e-016, 20}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, origin = {-80, -68}, rotation = -90), Rectangle(extent = {{-23, 14}, {23, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {-60, -111}, rotation = 90), Ellipse(extent = {{-29, 29}, {29, -29}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {-59, -131}, rotation = 90), Rectangle(extent = {{-23, 14}, {23, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {62, -43}, rotation = 90), Ellipse(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {62, -96}, rotation = 90), Text(extent = {{-12, 28}, {14, -28}}, lineColor = {255, 255, 255}, textString = "L"), Text(extent = {{-80, -110}, {-36, -154}}, lineColor = {255, 255, 255}, textString = "C1"), Text(extent = {{50, -66}, {76, -122}}, lineColor = {255, 255, 255}, textString = "C"), Text(extent = {{-42, -52}, {-10, -84}}, lineColor = {0, 0, 0}, textString = "R")}));
    end Nordergraaf;

    model Windkessel_3_Elem
      extends Basics.Interfaces.BloodStreamTwoPin;
      Resistance Rs(R = R) annotation(
        Placement(transformation(extent = {{4, -10}, {24, 10}})));
      Compliance compliance(C = C) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-14, 10})));
      Resistance Rz(R = Z) annotation(
        Placement(transformation(extent = {{-52, -10}, {-32, 10}})));
      parameter Basics.Units.HydraulicResistance Z = 0.015;
      parameter Basics.Units.Compliancy C = 1.5;
      parameter Basics.Units.HydraulicResistance R = 1.5;
    equation
      connect(cnStreamOut, cnStreamOut) annotation(
        Line(points = {{100, 0}, {100, 6}, {100, 6}, {100, 0}}, color = {255, 0, 0}, thickness = 0.5));
      connect(Rs.cnStreamIn, compliance.cnBloodStream) annotation(
        Line(points = {{4, 0}, {-14, 0}}, color = {255, 0, 0}, thickness = 0.5));
      connect(Rz.cnStreamOut, compliance.cnBloodStream) annotation(
        Line(points = {{-32, 0}, {-14, 0}}, color = {255, 0, 0}, thickness = 0.5));
      connect(Rs.cnStreamOut, cnStreamOut) annotation(
        Line(points = {{24, 0}, {100, 0}}, color = {255, 0, 0}, thickness = 0.5));
      connect(Rz.cnStreamIn, cnStreamIn) annotation(
        Line(points = {{-52, 0}, {-76, 0}, {-100, 0}}, color = {255, 0, 0}, thickness = 0.5));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-14, 14}, {14, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {-86, 0}, rotation = 180), Polygon(points = {{-1.14424e-016, 20}, {20, 40}, {20, 3.36469e-016}, {-1.14424e-016, 20}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, origin = {-52, 20}, rotation = 180), Polygon(points = {{-1.14424e-016, 20}, {20, 40}, {20, 3.36469e-016}, {-1.14424e-016, 20}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, origin = {-52, -20}, rotation = 0), Polygon(points = {{-1.14424e-016, 20}, {20, 40}, {20, 3.36469e-016}, {-1.14424e-016, 20}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, origin = {44, 48}, rotation = 180), Polygon(points = {{-1.14424e-016, 20}, {20, 40}, {20, 3.36469e-016}, {-1.14424e-016, 20}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, origin = {44, 8}, rotation = 0), Ellipse(extent = {{22, -6}, {66, -50}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}), Line(points = {{44, -6}, {44, -50}}, color = {0, 0, 0}, thickness = 0.5), Rectangle(extent = {{-18, 14}, {18, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {6, 28}, rotation = 180), Rectangle(extent = {{-20, 14}, {20, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {8, -28}, rotation = 180), Rectangle(extent = {{-20, 14}, {20, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {80, -28}, rotation = 180), Rectangle(extent = {{-18, 14}, {18, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {82, 28}, rotation = 180), Rectangle(extent = {{-14, 14}, {14, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {86, -1.77636e-015}, rotation = 270), Rectangle(extent = {{-14, 14}, {14, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {2, 0}, rotation = 270), Rectangle(extent = {{-10, 14}, {10, -14}}, lineColor = {0, 0, 0}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {255, 0, 0}, origin = {-22, 0}, rotation = 180)}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end Windkessel_3_Elem;

    model Compliance_VariableUnstressedVolume
      extends Basics.Icons.icoCompliance_OnePin;
      parameter Basics.Units.Compliancy C;
      parameter Basics.Units.Volume_ml V_0 = 0 "Unstressed volume";
      Basics.Units.Volume_ml V;
      Basics.Interfaces.cnBloodStream cnBloodStream annotation(
        Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(transformation(extent = {{86, -20}, {46, 20}}), iconTransformation(extent = {{86, -20}, {46, 20}})));
    equation
      der(V) = cnBloodStream.VolFlow;
      cnBloodStream.P = 1 / C * (V - (V_0 - u));
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end Compliance_VariableUnstressedVolume;

    model Resistance_init
      extends Resistance_Partial;
      parameter HumanLib.Basics.Units.Pressure_mmHg p_init = 135;
    initial equation
      cnStreamIn.P = p_init;
    equation
      dP = VolFlow * R;
    end Resistance_init;
  end Vessels;

  package Valves
    partial model Valve_Partial
      extends Basics.Icons.icoValve;
      extends Basics.Interfaces.BloodStreamOnePort;
      parameter Basics.Units.HydraulicResistance R;
    end Valve_Partial;

    model Valve_Switching "Valve in series connection with resistance. FlowRate is zero for cnStreamOut.P > cnStreamIn.P, 
                       resistance is proportional to cnStreamIn.P"
      extends Valve_Partial;
    equation
      VolFlow = dP * (if cnStreamIn.P + 1E-3 <= cnStreamOut.P then 10E-8 else 1 / R);
    end Valve_Switching;

    model Valve_Inertance
      extends Basics.Icons.icoValve;
      extends Basics.Interfaces.BloodStreamTwoPin;
      parameter Basics.Units.HydraulicResistance R = 0.003;
      parameter Basics.Units.HydraulicInertia L = 1e-4;
      Valve_Switching valve(R = R);
      Vessels.Inductance inductance(L = L);
    equation
      connect(cnStreamIn, valve.cnStreamIn);
      connect(valve.cnStreamOut, inductance.cnStreamIn);
      connect(inductance.cnStreamOut, cnStreamOut);
    end Valve_Inertance;

    model Valve_Logistic
      extends Valve_Partial;
      parameter Real k = 9;
    equation
      VolFlow = dP / R * (1 / (1 + exp(-k * dP)) + 0.001 * R);
    end Valve_Logistic;

    model Valve_Root
      extends Valve_Partial;
    equation
      VolFlow = dP / R * (2 * sqrt(1 + dP ^ 2) + 0.5);
    end Valve_Root;

    model Valve_Logistic_Tanh
      extends Valve_Partial;
    equation
      VolFlow = (1 - 1 / (1 + exp(-dP * 10000))) * tanh(dP * 3) + 1 / (1 + exp(-dP * 10000)) * dP / R;
    end Valve_Logistic_Tanh;

    partial model Valve_State_Partial
      //extends Valve_Partial;
      Real Valve_Open;
    end Valve_State_Partial;

    model Valve_State_Logistic
      extends Valve_Logistic;
      extends Valve_State_Partial;
    equation
      Valve_Open = 1 / (1 + exp(-k * dP));
    end Valve_State_Logistic;

    model Valve_State_Switching
      extends Valve_Switching;
      extends Valve_State_Partial;
    equation
      Valve_Open = if cnStreamIn.P <= cnStreamOut.P then 0 else 1;
    end Valve_State_Switching;

    model Valve_Variable_State_Logistic
      extends Valve_State_Partial;
      extends Valve_Partial_Variable;
      parameter Real k = 9;
    equation
      VolFlow = dP / (R + u) * (1 / (1 + exp(-k * dP)) + 0.001 * (R + u));
      Valve_Open = 1 / (1 + exp(-k * dP));
    end Valve_Variable_State_Logistic;

    model Valve_Partial_Variable
      extends Valve_Partial;
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(transformation(extent = {{20, -20}, {-20, 20}}, rotation = 90, origin = {0, 120})));
    end Valve_Partial_Variable;
  end Valves;

  package Heart
    partial model Ventricle_Elastance_Partial "Elastance based ventricle"
      extends Basics.Icons.icoCompliance_TwoPin;
      extends Basics.Interfaces.BloodStreamTwoPin;
      Basics.Units.Volume_ml Vol(min = 0);
      Basics.Interfaces.cnPV cnPV annotation(
        Placement(transformation(extent = {{0, 70}, {10, 80}}), iconTransformation(extent = {{-10, 60}, {10, 80}})));
    equation
      cnPV.V = Vol;
      cnStreamIn.P = cnPV.P;
      cnStreamOut.P = cnPV.P;
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-60, 60}, {60, -60}}, lineColor = {28, 108, 200}, lineThickness = 0.5)}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end Ventricle_Elastance_Partial;

    package TimeVaryingElastance
      partial model Norm_Time_Partial
        extends Modelica.Blocks.Icons.Block;
        parameter Real HeartRate = 60 "Frequency of the sawtooth wave.";
        Real T = 60 / HeartRate;
        parameter Modelica.SIunits.Time delay = 0;
        Modelica.SIunits.Time time_delayed;
        parameter Real offset = 0;
        Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
      equation
        time_delayed = time - delay;
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-80, 68}, {-80, -80}}, color = {192, 192, 192}), Polygon(points = {{-80, 90}, {-88, 68}, {-72, 68}, {-80, 90}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-90, -70}, {82, -70}}, color = {192, 192, 192}), Polygon(points = {{90, -70}, {68, -62}, {68, -78}, {90, -70}}, lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid), Line(points = {{-80, -70}, {-60, -70}, {0, 40}, {0, -70}, {60, 41}, {60, -70}}, color = {0, 0, 0})}),
          Diagram(graphics = {Polygon(points = {{-80, 90}, {-86, 68}, {-74, 68}, {-80, 90}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Line(points = {{-80, 68}, {-80, -80}}, color = {95, 95, 95}), Line(points = {{-90, -70}, {82, -70}}, color = {95, 95, 95}), Polygon(points = {{90, -70}, {68, -65}, {68, -75}, {90, -70}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Line(points = {{-10, -20}, {-10, -70}}, color = {95, 95, 95}), Line(points = {{-10, 46}, {-10, -62}}, color = {95, 95, 95}), Line(points = {{30, 46}, {30, 17}}, color = {95, 95, 95}), Line(points = {{-10, 41}, {30, 41}}, color = {95, 95, 95}), Text(extent = {{-12, 94}, {34, 85}}, lineColor = {0, 0, 0}, textString = "frequency"), Line(points = {{-44, 18}, {30, 18}}, color = {95, 95, 95}), Line(points = {{-34, 5}, {-34, -62}}, color = {95, 95, 95}), Text(extent = {{-73, -17}, {-36, -26}}, lineColor = {0, 0, 0}, textString = "1"), Polygon(points = {{-34, 18}, {-37, 5}, {-31, 5}, {-34, 18}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Polygon(points = {{-34, -62}, {-37, -49}, {-31, -49}, {-34, -62}, {-34, -62}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Polygon(points = {{-10, 41}, {-1, 43}, {-1, 39}, {-10, 41}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Polygon(points = {{30, 41}, {22, 43}, {22, 39}, {30, 41}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Line(points = {{-80, -62}, {-10, -62}, {30, 18}, {30, -62}, {72, 18}, {72, -62}}, color = {0, 0, 255}, thickness = 0.5)}));
      end Norm_Time_Partial;

      model Norm_Time
        extends Norm_Time_Partial;
      equation
        y = time_delayed / T - floor(time_delayed / T) + offset;
      end Norm_Time;

      model Norm_Time_Smooth "Approximation of the sawtooth using trigonomic functions"
        extends Norm_Time_Partial;
        parameter Real delta = 1e-5;
        //parameter pi = Modelica.Constants.pi;
        Real trg;
        Real sqr;
      equation
        trg = 1 - 2 * acos((1 - delta) * sin(2 * Modelica.Constants.pi * (2 * time_delayed - 1) / 4)) / Modelica.Constants.pi;
        sqr = 2 * atan(sin(2 * Modelica.Constants.pi * time_delayed / 2) / delta) / Modelica.Constants.pi;
        y = (1 + trg * sqr) / 2 + offset;
      end Norm_Time_Smooth;

      package ActivationFunctions
        partial block Activation_Partial
          extends Modelica.Blocks.Icons.Block;
          Modelica.Blocks.Interfaces.RealOutput phi annotation(
            Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
          parameter Modelica.SIunits.Time time_shift = 0;
          Modelica.Blocks.Interfaces.RealInput time_norm annotation(
            Placement(transformation(extent = {{-140, -20}, {-100, 20}}), iconTransformation(extent = {{-140, -20}, {-100, 20}})));
          annotation(
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-92, -72}, {-64, -72}, {-42, 16}, {-14, 34}, {20, 44}, {22, -18}, {42, -72}, {78, -72}}, color = {0, 0, 0}, thickness = 0.5, smooth = Smooth.Bezier), Line(points = {{-76, 66}, {-76, -82}}, color = {95, 95, 95}), Polygon(points = {{-76, 88}, {-82, 66}, {-70, 66}, {-76, 88}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Line(points = {{-86, -72}, {86, -72}}, color = {95, 95, 95}), Polygon(points = {{94, -72}, {72, -67}, {72, -77}, {94, -72}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid)}));
        end Activation_Partial;

        block Activation_Chung
          extends ActivationFunctions.Activation_Partial;
        equation
          phi = 0.9556 * exp(-255.4 * (time_norm - 0.3060) ^ 2) + 0.6249 * exp(-225.3 * (time_norm - 0.2026) ^ 2) + 0.0180 * exp(-4225.0 * (time_norm - 0.2491) ^ 2);
        end Activation_Chung;

        block Activation_Colacino
          extends ActivationFunctions.Activation_Partial;
          parameter Real T_2 = 0.5;
          parameter Real T_1 = 0.833 * T_2;
        equation
          phi = if time_norm <= T_1 then 0.5 + 0.5 * cos(Modelica.Constants.pi - Modelica.Constants.pi / T_1 * time_norm) elseif time_norm <= T_2 then 0.5 + 0.5 * cos(Modelica.Constants.pi * T_1 / (T_1 - T_2) - Modelica.Constants.pi * time_norm / (T_1 - T_2)) else 0;
          annotation(
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
        end Activation_Colacino;

        block Activation_Gauss5 "Identified from measurement data"
          extends ActivationFunctions.Activation_Partial;
          parameter Real A1 = -1.637489e+01;
          parameter Real B1 = 2.279919e-01;
          parameter Real C1 = 9.524781e-02;
          parameter Real A2 = 2.560582e-01;
          parameter Real B2 = 2.518663e-01;
          parameter Real C2 = 4.046358e-02;
          parameter Real A3 = 6.579947e-01;
          parameter Real B3 = 2.099908e-01;
          parameter Real C3 = 5.217069e-02;
          parameter Real A4 = 1.650815e+01;
          parameter Real B4 = 2.293927e-01;
          parameter Real C4 = 1.012489e-01;
          parameter Real A5 = 1.662089e-01;
          parameter Real B5 = 5.914052e-02;
          parameter Real C5 = 4.036142e-02;
        equation
//phi=-1.637489e+01*exp(-((time_norm-2.279919e-01)/9.524781e-02)^2)+2.560582e-01*exp(-((time_norm-2.518663e-01)/4.046358e-02)^2)+6.579947e-01*exp(-((time_norm-2.099908e-01)/5.217069e-02)^2)+1.650815e+01*exp(-((time_norm-2.293927e-01)/1.012489e-01)^2)+1.662089e-01*exp(-((time_norm-5.914052e-02)/4.036142e-02)^2);
          phi = 1 / 1.005455e+00 * (A1 * exp(-((time_norm - B1) / C1) ^ 2) + A2 * exp(-((time_norm - B2) / C2) ^ 2) + A3 * exp(-((time_norm - B3) / C3) ^ 2) + A4 * exp(-((time_norm - B4) / C4) ^ 2) + A5 * exp(-((time_norm - B5) / C5) ^ 2));
        end Activation_Gauss5;

        block Activation_Winkler
          extends ActivationFunctions.Activation_Partial;
          parameter Real Tir = 9;
          parameter Real Tsys = 9;
          parameter Real pi = 2 * Modelica.Math.asin(1.0);
        equation
          if Tsys + Tir < time_norm then
            phi = 0;
          elseif Tsys < time_norm and time_norm < Tsys + Tir then
            phi = 0.5 * (1 + cos(pi * ((time_norm - Tsys) / Tir)));
          elseif 0 < time_norm and time_norm < Tsys then
            phi = 0.5 * (1 - cos(pi * (time_norm / Tsys)));
          else
            phi = 0;
          end if;
        end Activation_Winkler;

        block Activation_ev_Neidlin
          extends ActivationFunctions.Activation_Partial;
          parameter Real Ts1 = 0.26;
          parameter Real Ts2 = 0.39;
          parameter Real pi = 2 * Modelica.Math.asin(1.0);
        equation
          if time_norm < Ts1 then
            phi = 0.5 * (1 - cos(time_norm / Ts1 * pi));
          elseif Ts1 <= time_norm and time_norm < Ts2 then
            phi = 0.5 * (1 + cos((time_norm - Ts1) / (Ts2 - Ts1) * pi));
          else
            phi = 0;
          end if;
//    if mod(time(n),0.8*1)<Ts1
//    e_v(n) = 1/2*(1-cos(mod(time(n),0.8*1)/Ts1*pi));
//    end
//    if mod(time(n),0.8*1)>=Ts1&&mod(time(n),0.8*1)<Ts2
//    e_v(n) = 1/2*(1+cos((mod(time(n),0.8*1)-Ts1)/(Ts2-Ts1)*pi));
//    end
//    if mod(time(n),0.8*1)>=Ts2
//    e_v(n) = 0;
//    end
        end Activation_ev_Neidlin;

        block Activation_ea_Neidlin
          extends ActivationFunctions.Activation_Partial;
          parameter Real Tas1 = 0.04;
          parameter Real Tas2 = 0.09;
          parameter Real Delta_T = 0.00;
          parameter Real pi = 2 * Modelica.Math.asin(1.0);
        equation
          if time_norm + Delta_T < Tas1 then
            phi = 0.5 * (1 - cos((time_norm + Delta_T) / Tas1 * pi));
          elseif Tas1 <= time_norm + Delta_T and time_norm + Delta_T < Tas2 then
            phi = 0.5 * (1 + cos((time_norm + Delta_T - Tas1) / (Tas2 - Tas1) * pi));
          else
            phi = 0;
          end if;
    //if mod((time_norm + Delta_T),0.8)< Tas1 then
    //phi = 0.5 * (1 - cos(mod((time_norm + Delta_T),0.8) / Tas1 * pi));
  //elseif Tas1 <= mod((time_norm + Delta_T),0.8) and mod((time_norm + Delta_T),0.8) < Tas2 then
    //phi = 0.5 * (1 + cos((mod((time_norm + Delta_T),0.8) - Tas1) / (Tas2 - Tas1) * pi));
  //else
    //phi = 0;
  //end if;
// Matlab code
//    if mod(time(n)+Delta_T,0.8)<Tas1
//    e_a(n) = 1/2*(1-cos(mod(time(n)+Delta_T,0.8)/Tas1*pi));
//    end
//    if mod(time(n)+Delta_T,0.8)>=Tas1&&mod(time(n)+Delta_T,0.8)<Tas2
//    e_a(n) = 1/2*(1+cos((mod(time(n)+Delta_T,0.8)-Tas1)/(Tas2-Tas1)*pi));
//    end
//    if mod(time(n)+Delta_T,0.8)>=Tas2
//    e_a(n) = 0;
//    end
        end Activation_ea_Neidlin;

        //    if mod(time(n)+Delta_T,0.8)<Tas1
        //    e_a(n) = 1/2*(1-cos(mod(time(n)+Delta_T,0.8)/Tas1*pi));
        //    end
        //    if mod(time(n)+Delta_T,0.8)>=Tas1&&mod(time(n)+Delta_T,0.8)<Tas2
        //    e_a(n) = 1/2*(1+cos((mod(time(n)+Delta_T,0.8)-Tas1)/(Tas2-Tas1)*pi));
        //    end
        //    if mod(time(n)+Delta_T,0.8)>=Tas2
        //    e_a(n) = 0;
        //    end
        //    if mod(time(n),0.8*1)<Ts1
        //    e_v(n) = 1/2*(1-cos(mod(time(n),0.8*1)/Ts1*pi));
        //    end
        //    if mod(time(n),0.8*1)>=Ts1&&mod(time(n),0.8*1)<Ts2
        //    e_v(n) = 1/2*(1+cos((mod(time(n),0.8*1)-Ts1)/(Ts2-Ts1)*pi));
        //    end
        //    if mod(time(n),0.8*1)>=Ts2
        //    e_v(n) = 0;
        //    end
      end ActivationFunctions;

      package ElastanceFunctions
        partial model Elastance_Partial
          extends Modelica.Blocks.Icons.Block;
          Modelica.Blocks.Interfaces.RealInput phi annotation(
            Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
          Basics.Interfaces.cnPV cnPV annotation(
            Placement(transformation(extent = {{100, -10}, {120, 10}})));
        equation

          annotation(
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-84, -66}, {-50, 52}, {58, 78}}, color = {0, 0, 0}, thickness = 0.5, smooth = Smooth.Bezier), Line(points = {{-70, -62}, {-4, -58}, {78, -40}}, color = {0, 0, 0}, thickness = 0.5, smooth = Smooth.Bezier), Polygon(points = {{60, -44}, {60, -32}, {60, 12}, {54, 36}, {-30, 64}, {-28, 20}, {-26, -40}, {-26, -62}, {50, -46}, {60, -44}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.HorizontalCylinder, smooth = Smooth.Bezier, fillColor = {0, 128, 255})}),
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
        end Elastance_Partial;

        model Elastance_Chung
          extends ElastanceFunctions.Elastance_Partial;
          parameter HumanLib.Basics.Units.Volume_ml V_d = 4;
          parameter HumanLib.Basics.Units.Elastance E_es = 7.7;
          parameter HumanLib.Basics.Units.Pressure_mmHg P_0 = 2.24;
          parameter Real lambda = 0.0583;
          parameter HumanLib.Basics.Units.Volume_ml V_0 = 5;
          HumanLib.Basics.Units.Pressure_mmHg P_es;
          HumanLib.Basics.Units.Pressure_mmHg P_ed;
        equation
          P_es = E_es * (cnPV.V - V_d);
          P_ed = P_0 * (exp(lambda * (cnPV.V - V_0)) - 1);
          cnPV.P = phi * P_es + (1 - phi) * P_ed;
          annotation(
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
        end Elastance_Chung;

        model Elastance_Colacino
          extends ElastanceFunctions.Elastance_Partial;
          Real phi_;
          Real phi_p;
          Real phi_a;
          parameter Real E_min = 0.025;
          parameter Real V_0 = 10;
          parameter Real K = 1500;
          parameter Real V_sat = 270;
          parameter Real V_stern = 175;
          parameter Real P_stern = 80;
          parameter Real P_0 = 0;
          parameter Real R_i = 0.03;
        equation
          phi_p = E_min * (cnPV.V - V_0) + K / (V_sat - (cnPV.V - V_0)) - K / V_sat;
          phi_a = (1 - ((V_stern - cnPV.V) / (V_stern - V_0)) ^ 2) * P_stern;
          phi_ = phi_p + (phi_a - phi_p) * phi;
          cnPV.P = P_0 + phi_ + R_i * der(cnPV.V);
          annotation(
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
        end Elastance_Colacino;

        partial model Elastance_Plugs_Base
          extends ElastanceFunctions.Elastance_Partial;
          Basics.Interfaces.cnPV cnPV_ESPVR annotation(
            Placement(transformation(extent = {{-120, 50}, {-100, 70}}), iconTransformation(extent = {{-120, 50}, {-100, 70}})));
          Basics.Interfaces.cnPV cnPV_EDPVR annotation(
            Placement(transformation(extent = {{-120, -70}, {-100, -50}}), iconTransformation(extent = {{-120, -70}, {-100, -50}})));
          annotation(
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-52, 7}, {52, -7}}, lineColor = {28, 108, 200}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, origin = {-36, 79}, rotation = 180, textString = "ESPVR", horizontalAlignment = TextAlignment.Left), Text(extent = {{-85, 8}, {85, -8}}, lineColor = {28, 108, 200}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid, origin = {-1, -82}, rotation = 180, textString = "EDPVR", horizontalAlignment = TextAlignment.Left)}));
        end Elastance_Plugs_Base;

        model Elastance_Plugs
          extends ElastanceFunctions.Elastance_Plugs_Base;
        equation
          cnPV.P = phi * cnPV_ESPVR.P + (1 - phi) * cnPV_EDPVR.P;
          cnPV_EDPVR.V = cnPV.V;
          cnPV_ESPVR.V = cnPV.V;
        end Elastance_Plugs;

        model Elastance_Plugs_Limit
          extends ElastanceFunctions.Elastance_Plugs_Base;
        equation
          cnPV_EDPVR.V = cnPV.V;
          cnPV_ESPVR.V = cnPV.V;
          cnPV.P = if cnPV_ESPVR.P >= cnPV_EDPVR.P then phi * cnPV_ESPVR.P + (1 - phi) * cnPV_EDPVR.P else cnPV_EDPVR.P;
        end Elastance_Plugs_Limit;

        model Elastance_haeger
          extends ElastanceFunctions.Elastance_Partial;
          parameter HumanLib.Basics.Units.Volume_ml Vd = 4;
          parameter HumanLib.Basics.Units.Elastance Emax = 7;
          parameter HumanLib.Basics.Units.Elastance Emin = 4;
        equation
          cnPV.P = (phi * (Emax - Emin) + Emin) * (cnPV.V - Vd);
          annotation(
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
        end Elastance_haeger;
        
        model Elastance_ventricle_Neidlin
          extends ElastanceFunctions.Elastance_Partial;
          parameter HumanLib.Basics.Units.Volume_ml V0 = 15;
          parameter HumanLib.Basics.Units.Elastance Emax = 2.8;
          parameter Real alpha = 0.064;
          parameter Real beta = 2.5;
          parameter Real kappa = 0.033;
        equation
          cnPV.P = phi * Emax * (cnPV.V - V0) + (1 - phi) * (alpha * exp(kappa * (cnPV.V - V0)) + beta);
          annotation(
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
        end Elastance_ventricle_Neidlin;
        
        model Elastance_atrium_Neidlin
          extends ElastanceFunctions.Elastance_Partial;
          parameter HumanLib.Basics.Units.Volume_ml V0 = 5;
          parameter HumanLib.Basics.Units.Elastance Emax = 0.45;
          parameter HumanLib.Basics.Units.Elastance Emin = 0.2125;
        equation
          cnPV.P = phi * Emax * (cnPV.V - V0) + (1 - phi) * Emin * (cnPV.V - V0);
          annotation(
            Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
        end Elastance_atrium_Neidlin;
      end ElastanceFunctions;

      package PV_Relationships
        partial model PV_Partial
          Basics.Interfaces.cnPV cnPV annotation(
            Placement(transformation(extent = {{100, -24}, {144, 20}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
          annotation(
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}}), graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, lineColor = {0, 0, 0}, lineThickness = 0.5), Line(points = {{-90, 16}, {-90, -36}}, color = {95, 95, 95}), Polygon(points = {{-90, 38}, {-96, 16}, {-84, 16}, {-90, 38}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid), Line(points = {{-96, -26}, {76, -26}}, color = {95, 95, 95}), Polygon(points = {{84, -26}, {62, -21}, {62, -31}, {84, -26}}, lineColor = {95, 95, 95}, fillColor = {95, 95, 95}, fillPattern = FillPattern.Solid)}),
            Diagram(coordinateSystem(extent = {{-100, -40}, {100, 40}})));
        end PV_Partial;

        model PV_Linear "P=E*(V-V_0)"
          extends PV_Partial;
          parameter Basics.Units.Elastance E = 1;
          parameter Basics.Units.Volume_ml V_0 = 10;
        equation
          cnPV.P = E * (cnPV.V - V_0);
          annotation(
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}}), graphics = {Line(points = {{-86, -30}, {74, 28}}, color = {95, 95, 95}, thickness = 0.5)}));
        end PV_Linear;

        model PV_Exponential "P=P_0*(exp(lambda*(cnPV.V-V_0))-offset)"
          extends PV_Partial;
          parameter HumanLib.Basics.Units.Pressure_mmHg P_0 = 2.24;
          parameter Real lambda = 0.0583;
          parameter HumanLib.Basics.Units.Volume_ml V_0 = 5;
          parameter Real offset = 1;
        equation
          cnPV.P = P_0 * (exp(lambda * (cnPV.V - V_0)) - offset);
          annotation(
            Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}}), graphics = {Line(points = {{-86, -24}, {-30, -24}, {16, -20}, {54, -4}, {66, 28}}, color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.Bezier)}));
        end PV_Exponential;

        model PV_Parabolic "P=(1-((V_max-V)/(V_max-V_0))^2)*P_max;"
          extends PV_Partial;
          parameter Basics.Units.Volume_ml V_max = 175;
          parameter Basics.Units.Pressure_mmHg P_max = 80;
          parameter Basics.Units.Volume_ml V_0 = 10;
        equation
          cnPV.P = (1 - ((V_max - cnPV.V) / (V_max - V_0)) ^ 2) * P_max;
          annotation(
            Icon(graphics = {Line(points = {{-76, -20}, {-16, 44}, {30, 0}}, color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.Bezier)}));
        end PV_Parabolic;

        model PV_Poly2 "P=a*V^2+b*V+c"
          extends PV_Partial;
          parameter Real a = -0.002938476;
          parameter Real b = 1.028466483;
          parameter Real c = -9.990817264;
        equation
          cnPV.P = a * cnPV.V ^ 2 + b * cnPV.V + c;
          annotation(
            Icon(graphics = {Line(points = {{-76, -20}, {-16, 44}, {30, 0}}, color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.Bezier)}));
        end PV_Poly2;
      end PV_Relationships;
    end TimeVaryingElastance;

    model Ventricle_Elastance_Simple
      extends Ventricle_Elastance_Partial;
    equation
      der(Vol) = cnStreamIn.VolFlow + cnStreamOut.VolFlow;
    end Ventricle_Elastance_Simple;

    model Ventricle_Elastance_VADSuction
      extends Ventricle_Elastance_Partial(Vol(start = 200));
      Basics.Interfaces.cnBloodStreamOut cnStreamOutVAD annotation(
        Placement(transformation(extent = {{-10, -110}, {10, -90}}, rotation = 0)));
      Basics.Units.HydraulicResistance R_Suction;
    equation
      der(Vol) = cnStreamIn.VolFlow + cnStreamOut.VolFlow + cnStreamOutVAD.VolFlow;
      R_Suction = exp(-(Vol - 5));
      cnStreamOut.P - cnStreamOutVAD.P = -R_Suction * cnStreamOutVAD.VolFlow;
    end Ventricle_Elastance_VADSuction;

    model Ventricle_Elastance_Simple_Init
      extends Ventricle_Elastance_Partial;
      parameter HumanLib.Basics.Units.Volume_ml V_init = 135;
    initial equation
      Vol = V_init;
    equation
      der(Vol) = cnStreamIn.VolFlow + cnStreamOut.VolFlow;
    end Ventricle_Elastance_Simple_Init;
  end Heart;

  package VAD
    model IdealFlowVAD
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoVAD;
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(transformation(extent = {{-20, 20}, {20, -20}}, rotation = 90, origin = {0, -94})));
    equation
      VolFlow = u;
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end IdealFlowVAD;

    model InertiaFlowVAD
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoVAD;
      parameter Real tau = 0.1;
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(transformation(extent = {{-20, 20}, {20, -20}}, rotation = 90, origin = {0, -94}), iconTransformation(extent = {{-20, 20}, {20, -20}}, rotation = 90, origin = {0, -120})));
    equation
      der(VolFlow) * tau + VolFlow = u;
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end InertiaFlowVAD;

    model DP3Hannes
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoVAD;
      Modelica.Blocks.Interfaces.RealInput omega_set_rad annotation(
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -104})));
      Basics.Units.HydraulicInertia L = 0.06;
      //VAD hydraulic inertia
      parameter Real c_1 = 0.160 "parameter of pump behaviour [mmHg.s/ml]";
      parameter Real c_2 = 1.76e-3 "parameter of pump behaviour [mmHg.s2/ml2]";
      parameter Real alpha = 0.536e-3 "parameter of pump behaviour [mmHg.s2/rad2]";
      parameter Real tau_vad = 0.22;
      Basics.Units.Pressure_mmHg p_head;
      Modelica.SIunits.AngularVelocity omega_rad;
      Real omega_RPM;
    equation
      VolFlow = (p_head + dP) / L;
//hydraulic inertance
      p_head = (-c_1 * VolFlow) - c_2 * abs(VolFlow) * VolFlow + alpha * omega_rad ^ 2;
//stationary
      der(omega_rad) = (omega_set_rad - omega_rad) / tau_vad;
      omega_RPM = 60 * omega_rad / (2 * Modelica.Constants.pi);
//maxon controller dynamics
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end DP3Hannes;

    model DP3Ident_MedIt_1809
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoVAD;
      Modelica.Blocks.Interfaces.RealInput omega_set annotation(
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -104})));
      parameter Basics.Units.HydraulicInertia L = 0.06;
      //VAD hydraulic inertia
      Basics.Units.Pressure_mmHg p_head;
      Modelica.SIunits.AngularVelocity omega;
      parameter Real tau_escon = 0.009799;
      Real VolFlow_L_min;
      //initial equation
      //omega=omega_set;
      //-dP=+8.481859e-01+1.082778e-03*omega-5.900903e-01*VolFlow_L_min+5.475070e-06*omega^2-1.325685e-04*omega*VolFlow_L_min-1.242091e+00*VolFlow_L_min^2+4.706215e-07*omega^2*VolFlow_L_min-3.584520e-04*omega*VolFlow_L_min^2+9.594131e-02*VolFlow_L_min^3;
      //p_head=-dP;
    equation
      VolFlow_L_min = VolFlow * 60 / 1000;
//VolFlow_L_min=10;
      der(VolFlow) = (p_head + dP) / L;
//hydraulic inertance
      VolFlow_L_min = (-1.872473e+00) - 2.189540e-01 * p_head + 4.080835e-03 * omega - 4.403628e-04 * p_head ^ 2 + 6.433626e-05 * p_head * omega - 5.557316e-07 * omega ^ 2 + 6.630895e-08 * p_head ^ 2 * omega - 5.671204e-09 * p_head * omega ^ 2 + 5.710496e-11 * omega ^ 3;
      der(omega) = (omega_set - omega) / tau_escon;
//maxon controller dynamics
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end DP3Ident_MedIt_1809;

    model DP3Dummy
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoVAD;
      Modelica.Blocks.Interfaces.RealInput omega_set annotation(
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -104})));
      parameter Basics.Units.HydraulicInertia L = 0.06;
      //VAD hydraulic inertia
      Basics.Units.Pressure_mmHg p_head(start = 0);
      Modelica.SIunits.AngularVelocity omega(start = 0);
      parameter Real tau_escon = 0.009799;
      Real VolFlow_L_min(start = 0);
    initial equation
      der(VolFlow_L_min) = 300;
    equation
      VolFlow_L_min = VolFlow * 60 / 1000;
      VolFlow_L_min = omega;
//Dummyequation
      p_head = (+8.481859e-01) + 1.082778e-03 * omega - 5.900903e-01 * VolFlow_L_min + 5.475070e-06 * omega ^ 2 - 1.325685e-04 * omega * VolFlow_L_min - 1.242091e+00 * VolFlow_L_min ^ 2 + 4.706215e-07 * omega ^ 2 * VolFlow_L_min - 3.584520e-04 * omega * VolFlow_L_min ^ 2 + 9.594131e-02 * VolFlow_L_min ^ 3;
      der(omega) = (omega_set - omega) / tau_escon;
//maxon controller dynamics
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end DP3Dummy;

    model DP3Ident_MedIt_1809_NoHydrInertia
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoVAD;
      Modelica.Blocks.Interfaces.RealInput omega_set annotation(
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -104})));
      parameter Basics.Units.HydraulicInertia L = 0.06;
      //VAD hydraulic inertia
      Basics.Units.Pressure_mmHg p_head;
      Modelica.SIunits.AngularVelocity omega;
      parameter Real tau_escon = 0.009799;
      Real VolFlow_L_min;
      //initial equation
      //omega=omega_set;
      //-dP=+8.481859e-01+1.082778e-03*omega-5.900903e-01*VolFlow_L_min+5.475070e-06*omega^2-1.325685e-04*omega*VolFlow_L_min-1.242091e+00*VolFlow_L_min^2+4.706215e-07*omega^2*VolFlow_L_min-3.584520e-04*omega*VolFlow_L_min^2+9.594131e-02*VolFlow_L_min^3;
      //p_head=-dP;
    equation
      VolFlow_L_min = VolFlow * 60 / 1000;
//VolFlow_L_min=10;
      p_head = -dP;
//der(VolFlow) = (p_head + dP)/L;
//hydraulic inertance
      VolFlow_L_min = (-1.872473e+00) - 2.189540e-01 * p_head + 4.080835e-03 * omega - 4.403628e-04 * p_head ^ 2 + 6.433626e-05 * p_head * omega - 5.557316e-07 * omega ^ 2 + 6.630895e-08 * p_head ^ 2 * omega - 5.671204e-09 * p_head * omega ^ 2 + 5.710496e-11 * omega ^ 3;
      der(omega) = (omega_set - omega) / tau_escon;
//maxon controller dynamics
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end DP3Ident_MedIt_1809_NoHydrInertia;

    model DP3_MLMedIt_Static_NeuralNet
      extends Basics.Interfaces.BloodStreamOnePort;
      extends Basics.Icons.icoVAD;
      Modelica.Blocks.Interfaces.RealInput w_rpm annotation(
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -104})));
      Basics.Units.Pressure_mmHg p_head;
      Real VolFlow_l_min;
    equation
      p_head = -dP;
      VolFlow = VolFlow_l_min * 1000 / 60;
//--pasted from CreateModelicaCode.m
//AutoGenerated by CreateModelicaCode.m at Zeit
      VolFlow_l_min = (7.15959e-01 + (2 / (1 + exp(-2 * ((-7.46423e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 7.62367e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-1.45145e+00)))) - 1) * (-5.58994e-02) + (2 / (1 + exp(-2 * ((-1.51305e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 1.36315e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 1.63825e+00))) - 1) * 1.35797e-01 + (2 / (1 + exp(-2 * ((-3.23896e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 3.46549e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-3.14567e+00)))) - 1) * (-4.66947e-02) + (2 / (1 + exp(-2 * (4.99658e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-9.51330e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 1.28340e+01))) - 1) * 4.89064e-02 + (2 / (1 + exp(-2 * (7.26583e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-6.47389e-01) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-8.50925e+00)))) - 1) * 2.21954e-02 + (2 / (1 + exp(-2 * ((-6.75790e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 6.81471e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 8.85394e-01))) - 1) * 5.35681e-02 + (2 / (1 + exp(-2 * ((-4.25868e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 1.15728e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 7.23438e+00))) - 1) * (-1.71903e-02) + (2 / (1 + exp(-2 * ((-2.02192e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-4.86462e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 1.41788e+00))) - 1) * 4.85571e-02 + (2 / (1 + exp(-2 * (2.30055e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-6.93560e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-6.94934e-01)))) - 1) * 9.29933e-03 + (2 / (1 + exp(-2 * (1.82235e-01 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 6.35619e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-5.20861e+00)))) - 1) * 5.35893e-02 + (2 / (1 + exp(-2 * (3.05147e-01 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-1.50023e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-2.87676e+00)))) - 1) * (-4.63115e-02) + (2 / (1 + exp(-2 * ((-3.15381e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 5.19931e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-1.05739e+01)))) - 1) * (-1.41963e+00) + (2 / (1 + exp(-2 * ((-1.06274e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 4.30356e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 9.42345e+00))) - 1) * (-9.95842e-03) + (2 / (1 + exp(-2 * ((-7.55764e-01) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 6.12237e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 3.76439e+00))) - 1) * 6.18585e-03 + (2 / (1 + exp(-2 * (8.70443e-01 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-2.05598e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 6.78928e+00))) - 1) * 3.86742e-02 + (2 / (1 + exp(-2 * ((-3.21823e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 5.45075e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-1.09591e+01)))) - 1) * 1.30570e+00 + (2 / (1 + exp(-2 * (1.25457e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 7.49647e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-4.22535e+00)))) - 1) * 5.56337e-01 + (2 / (1 + exp(-2 * ((-1.66237e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-5.32650e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-2.89827e+00)))) - 1) * (-6.65527e-03) + (2 / (1 + exp(-2 * (1.14113e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 7.00229e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-3.78680e+00)))) - 1) * (-5.89719e-01) + (2 / (1 + exp(-2 * (4.17830e-01 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 8.40129e-01 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-1.30726e+00)))) - 1) * (-6.79651e-01) + (2 / (1 + exp(-2 * ((-3.38073e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-4.26402e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-6.32728e+00)))) - 1) * (-3.37533e-03) + (2 / (1 + exp(-2 * (3.95243e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 5.75651e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-4.70711e+00)))) - 1) * 5.70815e-02 + (2 / (1 + exp(-2 * (2.59802e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 6.46449e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-4.64825e+00)))) - 1) * 8.30288e-02 + (2 / (1 + exp(-2 * (3.40264e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 1.81445e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-4.17818e+00)))) - 1) * (-6.90896e-02) + (2 / (1 + exp(-2 * (6.48696e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-1.44129e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 8.04581e+00))) - 1) * 1.21832e-01 + (2 / (1 + exp(-2 * ((-5.92338e+00) + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * (-4.34194e+00) + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 5.23925e+00))) - 1) * (-3.66203e-02) + (2 / (1 + exp(-2 * (6.31163e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 6.76268e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-5.78783e+00)))) - 1) * 2.53205e-02 + (2 / (1 + exp(-2 * (2.84753e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 6.62516e-01 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * 2.05387e+00))) - 1) * (-5.78025e-01) + (2 / (1 + exp(-2 * (6.93808e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 6.80367e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-1.29999e+00)))) - 1) * 8.67422e-03 + (2 / (1 + exp(-2 * (7.07610e+00 + ((p_head - (-4.98295e+01)) * 8.00462e-03 + (-1.00000e+00)) * 2.12777e+00 + ((w_rpm - 5.08838e+02) * 2.10863e-04 + (-1.00000e+00)) * (-7.52300e+00)))) - 1) * (-4.77521e-02) - (-1.00000e+00)) / 1.13997e-01 + (-6.12423e+00);
//Auto Generated Code End
//--past
    end DP3_MLMedIt_Static_NeuralNet;

    model Cannulation
      extends Basics.Interfaces.BloodStreamTwoPin;
      parameter Basics.Units.HydraulicResistance R = 0.0751;
      parameter Real R_Turb = 0.0112;
      parameter Basics.Units.HydraulicInertia L = 0.169;
      Vessels.Resistance_Turbulent resistance_Turbulent(R = R, R_Turb = R_Turb) annotation(
        Placement(transformation(extent = {{-60, -20}, {-20, 20}})));
      Vessels.Inductance inductance(L = L) annotation(
        Placement(transformation(extent = {{20, -20}, {60, 20}})));
    equation
      connect(resistance_Turbulent.cnStreamIn, cnStreamIn) annotation(
        Line(points = {{-60, 0}, {-100, 0}}, color = {255, 0, 0}, thickness = 0.5));
      connect(resistance_Turbulent.cnStreamOut, inductance.cnStreamIn) annotation(
        Line(points = {{-20, 0}, {20, 0}}, color = {255, 0, 0}, thickness = 0.5));
      connect(inductance.cnStreamOut, cnStreamOut) annotation(
        Line(points = {{60, 0}, {100, 0}}, color = {255, 0, 0}, thickness = 0.5));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, lineColor = {0, 0, 0}, lineThickness = 0.5), Line(points = {{-20, 40}, {-20, -40}, {20, -18}, {20, 18}, {-20, 40}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{20, 18}, {0, 18}, {0, 14}, {20, 14}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{20, -18}, {0, -18}, {0, -14}, {20, -14}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{0, -2}, {0, 2}, {20, 2}, {20, -2}, {0, -2}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{0, 10}, {20, 10}, {20, 6}, {0, 6}, {0, 10}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{0, -6}, {20, -6}, {20, -10}, {0, -10}, {0, -6}}, color = {0, 0, 0}, thickness = 0.5)}),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end Cannulation;
  end VAD;

  package Autoregulation
    package Baroreflex
      model Vasoconstriction_PI
        Basics.Interfaces.cnBloodStream cnBloodStream annotation(
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(transformation(extent = {{90, -10}, {110, 10}})));
        Basics.Sensors.PressureSensor pressureSensor annotation(
          Placement(transformation(extent = {{-84, -10}, {-64, 10}})));
        parameter Real PI_Gain = 0.01 "PI_Gain";
        parameter Modelica.SIunits.Time PI_T = 1 "Time Constant in seconds (T>0 required)";
        parameter Basics.Units.HydraulicResistance R_0 "Open loop resistance.";
        parameter Basics.Units.Pressure_mmHg P_mean_set "Pressure set point.";
        Basics.Sensors.MovingAverage movingAverage(T = Avg_T, n = Avg_n, y_0 = P_mean_set) annotation(
          Placement(transformation(extent = {{-36, -10}, {-16, 10}})));
        parameter Modelica.SIunits.Time Avg_T = 2 "Time interval used for calculating mean value of the pressure";
        parameter Integer Avg_n = 40 "number of increments in Avg_T";
        parameter Modelica.SIunits.Time startTime = 3 "Time at which the loop is closed (baroreflex is activated)";
        ActivatingPI activatingPI(set_point = P_mean_set, startTime = startTime, PI_T = PI_T, PI_Gain = PI_Gain) annotation(
          Placement(transformation(extent = {{44, -10}, {64, 10}})));
      equation
        connect(y, y) annotation(
          Line(points = {{100, 0}, {100, 0}}, color = {0, 0, 127}));
        connect(cnBloodStream, pressureSensor.cnBloodStream) annotation(
          Line(points = {{-100, 0}, {-84, 0}}, color = {255, 0, 0}, thickness = 0.5));
        connect(pressureSensor.p, movingAverage.u) annotation(
          Line(points = {{-63.4, 0}, {-38, 0}}, color = {0, 0, 127}));
        connect(movingAverage.y, activatingPI.u) annotation(
          Line(points = {{-15, 0}, {42, 0}}, color = {0, 0, 127}));
        connect(y, activatingPI.y) annotation(
          Line(points = {{100, 0}, {82, 0}, {65, 0}}, color = {0, 0, 127}));
        annotation(
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
      end Vasoconstriction_PI;
    end Baroreflex;

    model ActivatingPI
      Modelica.Blocks.Sources.Step step(startTime = startTime, height = 1, offset = 0) annotation(
        Placement(transformation(extent = {{-78, -60}, {-58, -40}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 0.5) annotation(
        Placement(transformation(extent = {{-44, -60}, {-24, -40}})));
      Modelica.Blocks.Math.Feedback feedback annotation(
        Placement(transformation(extent = {{-60, 10}, {-40, -10}})));
      Modelica.Blocks.Sources.Constant const(k = set_point) annotation(
        Placement(transformation(extent = {{-80, 20}, {-60, 40}})));
      Modelica.Blocks.Continuous.PI PI(k = PI_Gain, T = PI_T) annotation(
        Placement(transformation(extent = {{34, -10}, {54, 10}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(transformation(extent = {{100, -10}, {120, 10}})));
      Modelica.Blocks.Math.Product product annotation(
        Placement(transformation(extent = {{0, -10}, {20, 10}})));
      Modelica.Blocks.Interfaces.RealInput u annotation(
        Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
      Modelica.Blocks.Math.Gain gain(k = -1) annotation(
        Placement(transformation(extent = {{68, -10}, {88, 10}})));
      parameter Real set_point = 1 "Set parameter for the control variable";
      parameter Modelica.SIunits.Time startTime = 0 "Time until the controler starts (y=0 if t<startTime)";
      parameter Modelica.SIunits.Time PI_T = 0.1 "PI Time Constant (T>0 required)";
      parameter Real PI_Gain = 1 "Gain";
    equation
      connect(step.y, firstOrder.u) annotation(
        Line(points = {{-57, -50}, {-46, -50}}, color = {0, 0, 127}));
      connect(firstOrder.y, product.u2) annotation(
        Line(points = {{-23, -50}, {-12, -50}, {-12, -6}, {-2, -6}}, color = {0, 0, 127}));
      connect(const.y, feedback.u2) annotation(
        Line(points = {{-59, 30}, {-50, 30}, {-50, 8}}, color = {0, 0, 127}));
      connect(y, y) annotation(
        Line(points = {{110, 0}, {110, 0}}, color = {0, 0, 127}));
      connect(product.y, PI.u) annotation(
        Line(points = {{21, 0}, {21, 0}, {32, 0}}, color = {0, 0, 127}));
      connect(feedback.y, product.u1) annotation(
        Line(points = {{-41, 0}, {-14, 0}, {-14, 6}, {-2, 6}}, color = {0, 0, 127}));
      connect(feedback.u1, u) annotation(
        Line(points = {{-58, 0}, {-58, 0}, {-120, 0}}, color = {0, 0, 127}));
      connect(PI.y, gain.u) annotation(
        Line(points = {{55, 0}, {60, 0}, {66, 0}}, color = {0, 0, 127}));
      connect(y, gain.y) annotation(
        Line(points = {{110, 0}, {100, 0}, {89, 0}}, color = {0, 0, 127}));
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end ActivatingPI;
  end Autoregulation;

  package Tools
    block StartDelay
      extends Modelica.Blocks.Interfaces.SISO;
      parameter Real start_value;
      parameter Modelica.SIunits.Time delayTime(start = 1) "Delay time of output with respect to input signal";
      TimeSwitch timeSwitch(Switch_Time = delayTime) annotation(
        Placement(transformation(extent = {{-4, -10}, {16, 10}})));
      Modelica.Blocks.Sources.Constant const(k = start_value) annotation(
        Placement(transformation(extent = {{-58, 20}, {-38, 40}})));
      Modelica.Blocks.Nonlinear.FixedDelay fixedDelay(delayTime = delayTime) annotation(
        Placement(transformation(extent = {{-58, -16}, {-38, 4}})));
    equation
      connect(timeSwitch.u1, const.y) annotation(
        Line(points = {{-6, 6}, {-22, 6}, {-22, 30}, {-37, 30}}, color = {0, 0, 127}));
      connect(timeSwitch.u2, fixedDelay.y) annotation(
        Line(points = {{-6, -6}, {-22, -6}, {-37, -6}}, color = {0, 0, 127}));
      connect(fixedDelay.u, u) annotation(
        Line(points = {{-60, -6}, {-82, -6}, {-82, 0}, {-120, 0}}, color = {0, 0, 127}));
      connect(timeSwitch.y, y) annotation(
        Line(points = {{17, 0}, {110, 0}}, color = {0, 0, 127}));
      annotation(
        Documentation(info = "<html>
<p>
The Input signal is delayed by a given time instant, or more precisely:
</p>
<pre>
   y = u(time - delayTime) for time &gt; time.start + delayTime
     = u(start_valie)       for time &le; time.start + delayTime
</pre>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100.0, -100.0}, {100.0, 100.0}}, initialScale = 0.1), graphics = {Text(extent = {{8.0, -142.0}, {8.0, -102.0}}, textString = "delayTime=%delayTime"), Line(points = {{-92.0, 0.0}, {-80.7, 34.2}, {-73.5, 53.1}, {-67.1, 66.4}, {-61.4, 74.6}, {-55.8, 79.1}, {-50.2, 79.8}, {-44.6, 76.6}, {-38.9, 69.7}, {-33.3, 59.4}, {-26.9, 44.1}, {-18.83, 21.2}, {-1.9, -30.8}, {5.3, -50.2}, {11.7, -64.2}, {17.3, -73.1}, {23.0, -78.4}, {28.6, -80.0}, {34.2, -77.6}, {39.9, -71.5}, {45.5, -61.9}, {51.9, -47.2}, {60.0, -24.8}, {68.0, 0.0}}, color = {0, 0, 127}, smooth = Smooth.Bezier), Line(points = {{-62.0, 0.0}, {-50.7, 34.2}, {-43.5, 53.1}, {-37.1, 66.4}, {-31.4, 74.6}, {-25.8, 79.1}, {-20.2, 79.8}, {-14.6, 76.6}, {-8.9, 69.7}, {-3.3, 59.4}, {3.1, 44.1}, {11.17, 21.2}, {28.1, -30.8}, {35.3, -50.2}, {41.7, -64.2}, {47.3, -73.1}, {53.0, -78.4}, {58.6, -80.0}, {64.2, -77.6}, {69.9, -71.5}, {75.5, -61.9}, {81.9, -47.2}, {90.0, -24.8}, {98.0, 0.0}}, color = {160, 160, 164}, smooth = Smooth.Bezier)}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end StartDelay;

    block TimeSwitch
      extends Modelica.Blocks.Interfaces.SI2SO;
      parameter Modelica.SIunits.Time Switch_Time "Time at which it is switched to input u2." annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-76, 58}, {78, 2}}, color = {0, 0, 0}, thickness = 0.5), Ellipse(extent = {{-90, 66}, {-78, 54}}, lineColor = {0, 0, 0}, lineThickness = 0.5), Ellipse(extent = {{-90, -52}, {-78, -64}}, lineColor = {0, 0, 0}, lineThickness = 0.5), Ellipse(extent = {{78, 6}, {90, -6}}, lineColor = {0, 0, 0}, lineThickness = 0.5)}));
    equation
      y = if time < Switch_Time then u1 else u2;
    end TimeSwitch;

    block ParameterProfile
      extends Modelica.Blocks.Interfaces.SO;
      parameter Real A = 2;
      parameter Real B = 3;
      parameter Real C = 1;
      parameter Modelica.SIunits.Time t_1 = 10;
      parameter Modelica.SIunits.Time t_2 = 12;
      parameter Modelica.SIunits.Time t_3 = 20;
      parameter Modelica.SIunits.Time t_4 = 25;
    equation
      y = if time < t_1 then A else if time < t_2 then A + (B - A) * (time - t_1) / (t_2 - t_1) else if time < t_3 then B else if time < t_4 then B + (C - B) * (time - t_3) / (t_4 - t_3) else C;
    end ParameterProfile;

    block TwoPoint_Ideal
      extends Modelica.Blocks.Interfaces.SISO;
    equation
      if u > 0.1 then
        y = 1;
      else
        y = 0;
      end if;
      annotation(
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
    end TwoPoint_Ideal;

    model StepProfile
      extends Modelica.Blocks.Interfaces.SO;
      parameter Real StartValue = 0;
      parameter Real StepValueInterval = 1;
      parameter Real StepTime = 1 "in seconds";
    equation
      y = floor(time / StepTime) * StepValueInterval + StartValue;
    end StepProfile;
  end Tools;
  annotation(
    uses(Modelica(version = "3.2.2")));
end HumanLib;