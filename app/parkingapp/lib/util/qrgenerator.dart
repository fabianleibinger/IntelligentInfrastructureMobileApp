import 'package:qr_flutter/qr_flutter.dart';

class QRCodeGenerator {
  scannableQR() {
    QrImage(
      data: "1234567890",
      version: QrVersions.auto,
      size: 200.0,
    );
  }
}
