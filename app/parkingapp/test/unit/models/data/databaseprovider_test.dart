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
import 'package:parkingapp/models/data/databaseprovider.dart';
import 'package:sqflite/sqflite.dart';

void main() {
  ChargeableVehicle _chargeable;
  StandardVehicle _standard;
  StandardVehicle _standardModify;

  setUp(() {
    TimeOfDay time = TimeOfDay(hour: 0, minute: 0);
    _chargeable = ChargeableVehicle(
        '1', '', '', 0, 0, 0, 0, 0, true, true, true, true, '', time, time);
    _standard = StandardVehicle('', '2', '', 0, 0, 0, 0, 0, true, true, true);
    _standardModify =
        StandardVehicle('', '2', '', 0, 0, 0, 0, 0, false, true, true);
    DatabaseProvider.db.createDatabase();
  });

  //test('wadw', () {
  //  DatabaseProvider.db
  //      .insert(_standard)
  //      .then((value) => expect(value, _standard));
  //  expect(DatabaseProvider.db.getVehicles(), [_standard]);
  //});
//
  //test('wdawd', () {
  //  DatabaseProvider.db
  //      .insert(_standard)
  //      .then((value) => expect(value, _standard));
  //  expect(DatabaseProvider.db.getVehicles(), [_standard]);
  //});
//
  //test('wadaw', () {
  //  DatabaseProvider.db
  //      .insert(_standard)
  //      .then((value) => expect(value, _standard));
  //  expect(DatabaseProvider.db.getVehicles(), [_standard]);
  //});

  tearDown(() {
    _chargeable = null;
    _standard = null;
    _standardModify = null;
  });
}
