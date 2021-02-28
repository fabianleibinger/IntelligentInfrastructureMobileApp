import 'package:flutter/material.dart';
import 'package:parkingapp/ui/editvehicle/editvehicle.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class AddVehicle extends StatefulWidget {
  @override
  _AddVehicleState createState() => _AddVehicleState();
}

class _AddVehicleState extends State<AddVehicle> {
  @override
  Widget build(BuildContext context) {
    //update vehicle in MainPage
    UpdateMainPageVehicle.setUp(context: context);
    return WillPopScope(
        onWillPop: () => UpdateMainPageVehicle.cleanUp(context: context),
        child: Scaffold(
          appBar: AppBar(
            title: Text(AppLocalizations.of(context).addVehicleTitle),
          ),
          //load form asynchronously to increase performance
          body: FutureBuilder(
            future: () async {
              return VehicleForm(
                  route: MaterialPageRoute(
                builder: (context) => MainPage(vehicle.inAppKey),
              ));
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
