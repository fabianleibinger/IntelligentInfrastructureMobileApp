import 'package:flutter/material.dart';
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
          children: [
            Center(
              child: scannableQR(vehicle.inAppKey),
            ),
            Center(child: Text(AppLocalizations.of(context).textQRPage)),
          ],
        ));
  }

  Widget scannableQR(String data) {
    return QrImage(
      data: data,
      version: QrVersions.auto,
      size: 200.0,
    );
  }
}
