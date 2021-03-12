import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/FirstStart/appconfiguration.dart';
import 'package:parkingapp/ui/settingspage/qrpage.dart';
import 'package:parkingapp/ui/settingspage/settingspage.dart';
import 'package:qr_code_scanner/qr_code_scanner.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class QRScanner extends StatefulWidget {
  static const String routeName = '/qrscanner';
  final String id;
  QRScanner({Key key, this.id}) : super(key: key);

  @override
  State<StatefulWidget> createState() => _QRScannerState();
}

class _QRScannerState extends State<QRScanner> {
  Barcode result;
  QRViewController controller;
  final GlobalKey qrKey = GlobalKey(debugLabel: 'QR');
  var scannedVehicle;

  // In order to get hot reload to work we need to pause the camera if the platform
  // is android, or resume the camera if the platform is iOS.
  @override
  void reassemble() {
    super.reassemble();
    if (Platform.isAndroid) {
      controller.pauseCamera();
    } else if (Platform.isIOS) {
      controller.resumeCamera();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text('Daten importieren')),
      body: Column(
        children: <Widget>[
          Expanded(
            flex: 5,
            child: QRView(
              key: qrKey,
              onQRViewCreated: _onQRViewCreated,
            ),
          ),
          Expanded(
            flex: 1,
            child: Center(
              child: (result != null)
                  ? ListTile(
                      title: Text(""),
                      onTap: () {
                        Navigator.push(
                            context,
                            MaterialPageRoute(
                                builder: (BuildContext context) =>
                                    SettingsPage()));
                      })
                  : Text(AppLocalizations.of(context).scanQRText),
            ),
          )
        ],
      ),
    );
  }

  void _onQRViewCreated(QRViewController controller) {
    this.controller = controller;
    controller.scannedDataStream.listen((scanData) {
      setState(() {
        result = scanData;
        //Map values = json.decode(scanData.toString());
        //scannedVehicle = Vehicle.fromMap(values);
        print(scanData.toString());
      });
    });
  }

  @override
  void dispose() {
    controller?.dispose();
    super.dispose();
  }

  Future<Widget> afterBuild() {
    return Navigator.pushNamedAndRemoveUntil(
        context, Routes.settings, (Route<dynamic> route) => false);
  }
}
