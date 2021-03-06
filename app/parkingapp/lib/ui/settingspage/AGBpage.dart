import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class AGB extends StatelessWidget {
  static const String routeName = '/agbpage';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('AGB und Nutzungsbedingungen', style: whiteHeader),
      ),
      body: ScrollableAGB(),
    );
  }
}

class ScrollableAGB extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(children: [
      /*Padding(
        padding: EdgeInsets.all(10),
      ),*/
      Expanded(
        child: SingleChildScrollView(
          child: Text(AppLocalizations.of(context).terms),
        ),
      ),
    ]);
  }
}
