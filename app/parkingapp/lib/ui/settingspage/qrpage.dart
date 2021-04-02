import 'package:flutter/material.dart';
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:qr_flutter/qr_flutter.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class QRPage extends StatelessWidget {
  static const routeName = '/qrpage';
  final Vehicle vehicle;
  const QRPage({Key key, this.vehicle}) : super(key: key);
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
    String data = createQRString();

    return QrImage(
      data: data,
      version: QrVersions.auto,
      size: 300.0,
    );
  }

  String createQRString() {
    String data = 'type:';

    if (vehicle.runtimeType == StandardVehicle) {
      data = data + 'standard,';
    } else if (vehicle.runtimeType == ChargeableVehicle) {
      data = data + 'chargeable,';
    }

    return data + _getCompleteData();
  }

  String _getCompleteData() {
    return vehicle.toMap().toString();
  }
}
