import 'package:flutter/material.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkOutPage extends StatefulWidget {
  static const String routeName = '/parkoutpage';

  const ParkOutPage({Key key}) : super(key: key);

  @override
  _ParkOutPageState createState() => _ParkOutPageState();
}

class _ParkOutPageState extends State<ParkOutPage> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkOut(context));
  }

  //TODO: switch _firstBuild to false in setState()

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkOut),
      floatingActionButton: FloatingActionButton.extended(
        //cancel or park out button, not clickable
        label: vehicle.parkedIn
            ? Text('Fahrzeug wird ausgeparkt')
            : Text('Einparkvorgang wird abgebrochen'),
        backgroundColor: grey,
        onPressed: () {},
      ),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: ListTile(
        title: Text(currentParkingGarage.name),
        subtitle: Text(currentParkingGarage.type.toShortString()),
      ),
    );
  }
}
