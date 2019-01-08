function StateVals = SetTrim(Op_trim)
%% SetTrim  Sets the trimmed values as stat

StateVals.CSPos = Op_trim.States(1).x;
StateVals.Eular = Op_trim.States(2).x;
StateVals.AngRates= Op_trim.States(3).x;
StateVals.Vb= Op_trim.States(4).x;
StateVals.Xe= Op_trim.States(5).x;
StateVals.Eta= Op_trim.States(6).x;
StateVals.EtaDot= Op_trim.States(7).x;
