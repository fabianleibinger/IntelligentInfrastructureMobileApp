import 'package:bloc_test/bloc_test.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/bloc/blocs/vehiclebloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/bloc/events/resetvehicles.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';
import 'package:parkingapp/bloc/events/updatevehicle.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';

void main() {
  ChargeableVehicle _chargeable;
  StandardVehicle _standard;
  StandardVehicle _standardModify;

  setUp(() {
    TimeOfDay time = TimeOfDay(hour: 0, minute: 0);
    _chargeable = ChargeableVehicle(
        '1', '', '', 0, 0, 0, 0, 0, true, true, true, true, '', time, time);
    _standard = StandardVehicle('2', '', '', 0, 0, 0, 0, 0, true, true, true);
    _standardModify =
        StandardVehicle('2', '', '', 0, 0, 0, 0, 0, false, true, true);
  });

  blocTest(
    'Set vehicles should set all vehicles in the bloc',
    build: () => VehicleBloc([]),
    act: (bloc) => bloc.add(SetVehicles([_standard, _chargeable])),
    expect: () => [
      [_standard, _chargeable]
    ],
  );

  blocTest(
    'Add vehicle should add the specific vehicle to the bloc',
    build: () => VehicleBloc([]),
    act: (bloc) => bloc.add(AddVehicle(_standard)),
    expect: () => [
      [_standard]
    ],
  );

  blocTest(
    'Reset vehicles should reset all vehicles in the bloc',
    build: () => VehicleBloc([_standard, _chargeable]),
    act: (bloc) => bloc.add(ResetVehicles()),
    expect: () => [[]],
  );

  blocTest(
    'Delete vehicle should delete a specific vehicle in the bloc',
    build: () => VehicleBloc([_standard, _chargeable]),
    act: (bloc) => bloc.add(DeleteVehicle(_chargeable)),
    expect: () => [
      [_standard]
    ],
  );

  blocTest(
    'Update vehicle should modify a specific vehicle in the bloc',
    build: () => VehicleBloc([_standard, _chargeable]),
    act: (bloc) => bloc.add(UpdateVehicle(_standardModify)),
    expect: () => [
      [_standardModify, _chargeable]
    ],
  );

  tearDown(() {
    _chargeable = null;
    _standard = null;
    _standardModify = null;
  });
}
