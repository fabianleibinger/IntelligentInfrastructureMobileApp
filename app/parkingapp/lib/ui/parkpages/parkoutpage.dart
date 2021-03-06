import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkOutPage extends StatefulWidget {
  static const String routeName = '/parkoutpage';

  final String carInAppKey;

  const ParkOutPage(this.carInAppKey);

  @override
  _ParkOutPageState createState() => _ParkOutPageState();
}

class _ParkOutPageState extends State<ParkOutPage> {
  @override
  void initState() {
    super.initState();
    //wait until build finished to call method
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkOut(context));
  }

  //TODO: switch _firstBuild to false in setState()

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkOut),
      //button observes parkedIn value of car
      floatingActionButton: ValueListenableBuilder(
        valueListenable: vehicle.parkedInObserver,
        builder: (BuildContext context, bool, Widget child) {
          return FloatingActionButton.extended(
            //cancel or park out process button, not clickable
            label: vehicle.parkedIn
                ? Text(AppLocalizations.of(context).actionButtonParkOutProcess)
                : Text(
                    AppLocalizations.of(context).actionButtonCancelParkProcess),
            backgroundColor: grey,
            onPressed: () {},
          );
        },
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: ListTile(
        title: Text(currentParkingGarage.name),
        subtitle: Text(currentParkingGarage.type.toShortString()),
      ),
    );
  }
}
