import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:qr_flutter/qr_flutter.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class QRPage extends StatelessWidget {
  static const routeName = '/qrpage';
  final Vehicle vehicle;
  const QRPage(Vehicle convertIntoVehicle, {Key key, this.vehicle})
      : super(key: key);
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text('Daten übertragen'),
        ),
        body: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Center(
              //vehicle.inAppKey kann noch nich übergeben werden
              child: scannableQR(vehicle),
            ),
            Padding(
              padding: EdgeInsets.only(top: 80.0),
            ),
            Padding(
              padding: EdgeInsets.all(7.5),
              child:
                  Center(child: Text(AppLocalizations.of(context).textQRPage)),
            ),
          ],
        ));
  }

  Widget scannableQR(Vehicle vehicle) {
    //if (vehicle != null) {
    String data;

    data = _getCompleteData();

    return QrImage(
      data: data,
      version: QrVersions.auto,
      size: 300.0,
    );
    /*} else {
      String data = "Nothing to show";
      return QrImage(
        data: data,
        version: QrVersions.auto,
        size: 300.0,
      );
    }*/
  }

  String _getCompleteData() {
    return vehicle.toMap().toString();
  }
}
