import 'package:flutter/material.dart';
import 'package:parkingapp/ui/FirstStart/routelandingpage.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/util/vehiclehelper.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// Add the first [Vehicle] during first start of the app
class AddVehicle extends StatefulWidget {
  @override
  _AddVehicleState createState() => _AddVehicleState();
}

class _AddVehicleState extends State<AddVehicle> {
  @override
  Widget build(BuildContext context) {
    //update vehicle in MainPage
    VehicleHelper.updateMainPageVehicle(context: context);
    SharedPreferences.getInstance().then(
        (preferences) => preferences.setBool(RouteLandingPage.isSetUp, true));
    return WillPopScope(
        onWillPop: () {
          SharedPreferences.getInstance().then((preferences) =>
              preferences.setBool(RouteLandingPage.isSetUp, false));
          return VehicleHelper.cleanUpDummy(context: context);
        },
        child: Scaffold(
          appBar: AppBar(
            title: Text(AppLocalizations.of(context).addVehicleTitle),
          ),
          //load form asynchronously to increase performance
          body: FutureBuilder(
            future: () async {
              return VehicleForm(
                route:
                    MaterialPageRoute(builder: (context) => RouteLandingPage()),
              );
            }(),
            builder:
                (BuildContext context, AsyncSnapshot<VehicleForm> snapshot) {
              if (snapshot.hasData)
                return snapshot.data;
              else
                return CircularProgressIndicator();
            },
          ),
        ));
  }
}
