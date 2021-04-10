import 'package:flutter/material.dart';
import 'package:parkingapp/models/data/sharedpreferences.dart';
import 'package:parkingapp/models/global.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

/// Creates scrollable page for conditions and terms of the app
class AGB extends StatelessWidget {
  static const String routeName = '/agbpage';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(AppLocalizations.of(context).termsAndConditionsTitle,
            style: whiteHeader),
      ),
      body: ScrollableAGB(),
    );
  }
}

class ScrollableAGB extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(children: [
      Expanded(
        child: SingleChildScrollView(
          child: Text(AppLocalizations.of(context).terms),
        ),
      ),
    ]);
  }
}
