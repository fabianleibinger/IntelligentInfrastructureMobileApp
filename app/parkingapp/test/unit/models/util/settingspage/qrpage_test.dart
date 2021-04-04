import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
<<<<<<< HEAD
import 'package:parkingapp/util/qrgenerator.dart';
=======
import 'package:parkingapp/ui/settingspage/qrpage.dart';
>>>>>>> refs/heads/build_ParkPage
import 'package:parkingapp/util/qrscanner.dart';

void main() {
  StandardVehicle _standard;
  ChargeableVehicle _chargeable;
  Vehicle _transferedVehicle;
  QRPage _qrpage;
  ScanState _qrscanner;
  String _qrstring;
  String _vehicletoMap;
  String _transferedVehicletoMap;
  setUp(() {
    _standard = StandardVehicle('', '', '', 0, 0, 0, 1, 0, true, true, false);
    TimeOfDay time = TimeOfDay(hour: 0, minute: 0);
    _chargeable = ChargeableVehicle(
        '', '', '', 0, 0, 0, 0, 0, true, true, true, true, '', time, time);
<<<<<<< HEAD
=======
    _qrpage = QRPage(vehicle: _standard);
>>>>>>> refs/heads/build_ParkPage
    _qrscanner = ScanState();
  });

  test('Equal standard vehicle after transfer', () {
<<<<<<< HEAD
    _qrpage = QRPage(vehicle: _standard);
    _qrstring = _qrpage.createQRString();
=======
    _qrstring = _qrpage.createQRString(_standard);
>>>>>>> refs/heads/build_ParkPage
    _transferedVehicle = _qrscanner.transferIntoVehicle(_qrstring);
    _vehicletoMap = _standard.toMap().toString();
    _transferedVehicletoMap = _transferedVehicle.toMap().toString();
    expect(_vehicletoMap == _transferedVehicletoMap, true);
    expect(_standard.equals(_transferedVehicle), true);
  });

  test('Equal chargeable vehicle after transfer', () {
<<<<<<< HEAD
    _qrpage = QRPage(vehicle: _chargeable);
    _qrstring = _qrpage.createQRString();
    _transferedVehicle = _qrscanner.transferIntoVehicle(_qrstring);
    _vehicletoMap = _chargeable.toMap().toString();
    _transferedVehicletoMap = _transferedVehicle.toMap().toString();
    expect(_chargeable.equals(_transferedVehicle), true);
=======
    _qrstring = _qrpage.createQRString(_chargeable);
    print(_chargeable.toMap().toString());
    print(_qrstring);
    _transferedVehicle = _qrscanner.transferIntoVehicle(_qrstring);
    _vehicletoMap = _standard.toString();
    _transferedVehicletoMap = _transferedVehicle.toString();
>>>>>>> refs/heads/build_ParkPage
    expect(_vehicletoMap == _transferedVehicletoMap, true);
  });

  tearDown(() {
    _qrstring = null;
    _standard = null;
    _chargeable = null;
    _transferedVehicletoMap = null;
    _vehicletoMap = null;
    _qrscanner = null;
    _qrpage = null;
    _transferedVehicle = null;
  });
}
