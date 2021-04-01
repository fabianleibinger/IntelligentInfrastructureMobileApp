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

class ScanScreen extends StatefulWidget {
  static const String routeName = '/qrscanner';
  @override
  _ScanState createState() => new _ScanState();
}

class _ScanState extends State<ScanScreen> {
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

  Future scan() async {
    try {
      String barcode = await BarcodeScanner.scan();
      setState(() {
        this.barcode = barcode;
        this.scanned = true;
        this.textButton =
            "Der QR Code wurde erfolgreich gescannt! Zum Fahrzeug hinzufügen hier klicken.";
      });
    } on PlatformException catch (e) {
      if (e.code == BarcodeScanner.CameraAccessDenied) {
        setState(() {
          this.barcode = 'The user did not grant the camera permission!';
        });
      } else {
        setState(() => this.barcode = 'Unknown error: $e');
      }
    } on FormatException {
      setState(() => this.barcode =
          'null (User returned using the "back"-button before scanning anything. Result)');
    } catch (e) {
      setState(() => this.barcode = 'Unknown error: $e');
    }
  }

  _addTransferedVehicle() {
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
        hour: int.parse(chargeTimeEnd[1]), minute: int.parse(chargeTimeEnd[2]));

    Vehicle vehicle;

    if (type[1] == 'chargeable') {
      vehicle = transferChargeableVehicle(
          inAppKey[1],
          name[1],
          licensePlate[1],
          double.tryParse(width[1]),
          double.tryParse(height[1]),
          double.tryParse(length[1]),
          double.tryParse(turningCycle[1]),
          double.tryParse(distRearAxleLicensePlate[1]),
          nearExitPreference[1] == '1',
          parkingCard[1] == '1',
          parkedIn[1] == '1',
          doCharge[1] == '1',
          chargingProvider[1],
          startTime,
          endTime);
    } else if (type[1] == 'standard') {
      vehicle = transferStandardVehicle(
          inAppKey[1],
          name[1],
          licensePlate[1],
          double.tryParse(width[1]),
          double.tryParse(height[1]),
          double.tryParse(length[1]),
          double.tryParse(turningCycle[1]),
          double.tryParse(distRearAxleLicensePlate[1]),
          nearExitPreference[1] == '1',
          parkingCard[1] == '1',
          parkedIn[1] == '1');
    }
    DataHelper.addVehicle(context, vehicle);
  }

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
