import 'package:flutter/material.dart';
import 'package:parkingapp/ui/FirstStart/termsofservice.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class LandingPage extends StatelessWidget {
  static const String routeName = '/LandingPage';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text(AppLocalizations.of(context).titleWelcome),
        ),
        bottomNavigationBar: Material(
          elevation: 10,
          child: ButtonBar(
            children: [
              TextButton(
                child: Text(AppLocalizations.of(context).buttonContinue),
                onPressed: () => Navigator.push(context,
                    MaterialPageRoute(builder: (context) => TermsOfService())),
              )
            ],
          ),
        ),
        body: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Center(
              child: Column(
                children: [
                  Icon(
                    Icons.directions_car,
                    size: 150,
                  ),
                  Text(AppLocalizations.of(context).appName)
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.all(10.0),
              child: Text(
                AppLocalizations.of(context).welcomeText,
                textAlign: TextAlign.justify,
              ),
            ),
          ],
        ));
  }
}
