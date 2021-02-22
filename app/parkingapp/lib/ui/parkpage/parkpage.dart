import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkPage extends StatefulWidget {
  static const String routeName = '/parkpage';

  const ParkPage({Key key}) : super(key: key);

  @override
  _ParkPageState createState() => _ParkPageState();
}

class _ParkPageState extends State<ParkPage> {
  @override
  void initState() {
    // TODO: implement initState
    super.initState();
    vehicle.parkIn(context);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.park),
      floatingActionButton: FloatingActionButton.extended(
        label: Text(AppLocalizations.of(context).actionButtonCancelPark),
        backgroundColor: red,
        onPressed: () {Navigator.pushReplacementNamed(context, vehicle.inAppKey);},
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: ListTile(
        //leading: Icon(Icons.location_on),
        title: Text(currentParkingGarage.name),
        subtitle:
        Text(currentParkingGarage.type.toShortString()),
      ),
    );
  }
}
