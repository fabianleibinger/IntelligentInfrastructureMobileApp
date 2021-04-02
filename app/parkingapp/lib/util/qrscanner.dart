import 'dart:async';

import 'package:barcode_scan/barcode_scan.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/models/data/datahelper.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/ui/vehiclepage/vehiclepage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

/// Class that creates barcode scanner and adds scanned vehicle to app
class ScanScreen extends StatefulWidget {
  static const String routeName = '/qrscanner';
  @override
  ScanState createState() => new ScanState();
}

class ScanState extends State<ScanScreen> {
  String barcode = "";
  String textButton = "Noch keinen QR Code gescannt!";
  bool scanned = false;

  @override
  initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: new AppBar(
          title: new Text('QR Code Scanner'),
        ),
        body: new Center(
          child: new Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[
              Padding(
                padding: EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
                child: ElevatedButton(
                    style: ElevatedButton.styleFrom(
                        primary: green, onPrimary: white, shadowColor: grey),
                    onPressed: scan,
                    child: const Text('Kamera zum Scannen starten')),
              ),
              Padding(
                padding: EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
                child: ElevatedButton(
                    style: ElevatedButton.styleFrom(
                        primary: grey, onPrimary: white, shadowColor: grey),
                    onPressed: () {
                      if (scanned) {
                        // add scanned vehicle to database
                        _addTransferedVehicle();
                        Navigator.push(
                            context,
                            MaterialPageRoute(
                                builder: (BuildContext context) =>
                                    VehiclePage()));
                      }
                    },
                    child: Text(
                      textButton,
                      textAlign: TextAlign.center,
                    )),
              ),
            ],
          ),
        ));
  }

  /// Opens barcode scanner
  Future scan() async {
    try {
      String barcode = await BarcodeScanner.scan();
      setState(() {
        this.barcode = barcode;
        this.scanned = true;
        this.textButton = AppLocalizations.of(context).barcodeScanned;
      });
    } on PlatformException catch (e) {
      if (e.code == BarcodeScanner.CameraAccessDenied) {
        setState(() {
          this.barcode = AppLocalizations.of(context).cameraAcessDenied;
        });
      } else {
        setState(
            () => this.barcode = AppLocalizations.of(context).unknownException);
      }
    } on FormatException {
      setState(
          () => this.barcode = AppLocalizations.of(context).unknownException);
    } catch (e) {
      setState(
          () => this.barcode = AppLocalizations.of(context).unknownException);
    }
  }

  /// Adds new vehicle object to database
  _addTransferedVehicle() {
    Vehicle vehicle = transferIntoVehicle(this.barcode);
    DataHelper.addVehicle(context, vehicle);
  }

  /// Creates new vehicle from String
  Vehicle transferIntoVehicle(String barcode) {
    //split the string coming from the QR Code
    String splitter = ':';
    List<String> split = barcode.split(',');
    List<String> type = split[0].split(splitter);
    List<String> inAppKey = split[1].split(splitter);
    List<String> name = split[2].split(splitter);
    List<String> licensePlate = split[3].split(splitter);
    List<String> width = split[4].split(splitter);
    List<String> height = split[5].split(splitter);
    List<String> length = split[6].split(splitter);
    List<String> turningCycle = split[7].split(splitter);
    List<String> distRearAxleLicensePlate = split[8].split(splitter);
    List<String> nearExitPreference = split[9].split(splitter);
    List<String> parkingCard = split[10].split(splitter);
    List<String> parkedIn = split[11].split(splitter);

    Vehicle vehicle;

    //check type of scanned vehicle
    if (type[1] == 'chargeable') {
      List<String> doCharge = split[12].split(splitter);
      List<String> chargingProvider = split[13].split(splitter);
      List<String> chargeTimeBegin = split[14].split(splitter);
      print(chargeTimeBegin);
      List<String> chargeTimeEnd = split[15].split(splitter);
      print(chargeTimeEnd);

      //convert String time into TimeOfDay
      TimeOfDay startTime = TimeOfDay(
          hour: int.parse(chargeTimeBegin[1]),
          minute: int.parse(chargeTimeBegin[2]));

      TimeOfDay endTime = TimeOfDay(
          hour: int.parse(chargeTimeEnd[1]),
          minute: int.parse(chargeTimeEnd[2].substring(0, 2)));

      vehicle = transferChargeableVehicle(
          inAppKey[1].substring(1),
          name[1].substring(1),
          licensePlate[1].substring(1),
          double.tryParse(width[1]),
          double.tryParse(height[1]),
          double.tryParse(length[1]),
          double.tryParse(turningCycle[1]),
          double.tryParse(distRearAxleLicensePlate[1]),
          nearExitPreference[1] == ' 1',
          parkingCard[1] == ' 1',
          parkedIn[1] == ' 1',
          doCharge[1] == ' 1',
          chargingProvider[1].substring(1),
          startTime,
          endTime);
    } else if (type[1] == 'standard') {
      vehicle = transferStandardVehicle(
          inAppKey[1].substring(1),
          name[1].substring(1),
          licensePlate[1].substring(1),
          double.tryParse(width[1]),
          double.tryParse(height[1]),
          double.tryParse(length[1]),
          double.tryParse(turningCycle[1]),
          double.tryParse(distRearAxleLicensePlate[1]),
          nearExitPreference[1] == ' 1',
          parkingCard[1] == ' 1',
          parkedIn[1] == ' 1');
    }
    return vehicle;
  }

  /// Creates new standard vehicle object
  Vehicle transferStandardVehicle(
      String inAppKey,
      String name,
      String licensePlate,
      double width,
      double height,
      double length,
      double turningCycle,
      double distRearAxleLicensePlate,
      bool nearExitPreference,
      bool parkingCard,
      bool parkedIn) {
    Vehicle vehicle = StandardVehicle(
        inAppKey,
        name,
        licensePlate,
        width,
        height,
        length,
        turningCycle,
        distRearAxleLicensePlate,
        nearExitPreference,
        parkingCard,
        parkedIn);
    return vehicle;
  }

  /// Creates chargeable vehicle object
  Vehicle transferChargeableVehicle(
      String inAppKey,
      String name,
      String licensePlate,
      double width,
      double height,
      double length,
      double turningCycle,
      double distRearAxleLicensePlate,
      bool nearExitPreference,
      bool parkingCard,
      bool parkedIn,
      bool doCharge,
      String chargingProvider,
      TimeOfDay chargeTimeBegin,
      TimeOfDay chargeTimeEnd) {
    Vehicle vehicle = ChargeableVehicle(
        inAppKey,
        name,
        licensePlate,
        width,
        height,
        length,
        turningCycle,
        distRearAxleLicensePlate,
        nearExitPreference,
        parkingCard,
        parkedIn,
        doCharge,
        chargingProvider,
        chargeTimeBegin,
        chargeTimeEnd);
    return vehicle;
  }
}
