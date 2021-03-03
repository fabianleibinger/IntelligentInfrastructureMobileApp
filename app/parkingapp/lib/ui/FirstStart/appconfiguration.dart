import 'package:flutter/material.dart';
import 'package:parkingapp/ui/FirstStart/addvehicle.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class AppConfiguration extends StatefulWidget {
  @override
  _AppConfigurationState createState() => _AppConfigurationState();
}

class _AppConfigurationState extends State<AppConfiguration> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text(AppLocalizations.of(context).titleConfigureApp),
        ),
        bottomNavigationBar: Material(
          elevation: 10,
          child: ButtonBar(
            children: [
              FlatButton(
                child: Text(AppLocalizations.of(context).buttonContinue),
                onPressed: () => Navigator.push(context,
                    MaterialPageRoute(builder: (context) => AddVehicle())),
              )
            ],
          ),
        ),
        body: Center(
          child: Text('TODO: Show settings page here'),
        ));
  }
}
