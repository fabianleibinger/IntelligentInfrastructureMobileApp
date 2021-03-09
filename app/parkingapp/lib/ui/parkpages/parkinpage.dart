import 'package:flutter/material.dart';
import 'package:parkingapp/dialogs/parkdialog.dart';
import 'package:parkingapp/models/global.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/appdrawer/appdrawer.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:parkingapp/models/enum/parkinggaragetype.dart';

class ParkInPage extends StatefulWidget {
  static const String routeName = '/parkinpage';

  const ParkInPage({Key key}) : super(key: key);

  @override
  _ParkInPageState createState() => _ParkInPageState();
}

class _ParkInPageState extends State<ParkInPage> {
  @override
  void initState() {
    super.initState();
    //wait until build finished to call method
    WidgetsBinding.instance
        .addPostFrameCallback((_) => vehicle.parkIn(context));
  }

  //TODO: setState when parkedIn switches

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(vehicle.name, style: whiteHeader)),
      drawer: AppDrawer(Routes.parkIn),
      //button observes parkedIn value of car
      floatingActionButton: ValueListenableBuilder(
          valueListenable: vehicle.parkedInObserver,
          builder: (BuildContext context, bool, Widget child) {
            return FloatingActionButton.extended(
              //cancel or park out button
              label: vehicle.parkedIn
                  ? Text(AppLocalizations.of(context).actionButtonParkOut)
                  : Text(AppLocalizations.of(context).actionButtonCancelPark),
              backgroundColor: red,
              //park out dialog or cancel dialog
              onPressed: vehicle.parkedIn
                  ? () => ParkDialog.createParkOutDialog(context)
                  : () => ParkDialog.createParkInCancelDialog(context),
            );
          }),
      floatingActionButtonLocation: FloatingActionButtonLocation.centerFloat,
      body: Column(
        children: [
          ListTile(
            title: Text(currentParkingGarage.name),
            subtitle: Text(currentParkingGarage.type.toShortString()),
          ),
          getParkInAnimation(
              context: context,
              bottomLeftLongitude: 8.41950527853,
              bottomLeftLattitude: 49.01388810447,
              topRightLattitude: 49.0144759205,
              topRightLongitude: 8.42059599234,
              lattitude: 49.01431771428,
              longitude: 8.42011294615)
        ],
      ),
    );
  }

  Container getParkInAnimation(
      {BuildContext context,
      double bottomLeftLongitude,
      double bottomLeftLattitude,
      double topRightLongitude,
      double topRightLattitude,
      double longitude,
      double lattitude}) {
    final _imageDirectory = "assets/parkgarage-fasanengarten-map.jpg";
    final AssetImage _garageImage = AssetImage(_imageDirectory);
    final double _height = 250;
    final double _width = MediaQuery.of(context).size.width;

    //factor by which the image is scaled to fit the width of the device
    //double _scaleFactor =
    //    MediaQuery.of(context).size.width / Image.asset(_imageDirectory).width;

    double _bottom = (_height /
        (topRightLattitude - bottomLeftLattitude) *
        (lattitude - bottomLeftLattitude));
    double _left = (_width / (topRightLongitude - bottomLeftLongitude)) *
        (longitude - bottomLeftLongitude);

    return Container(
        width: MediaQuery.of(context).size.width,
        height: _height, //this is the same as the main page image height
        child: Stack(
          children: [
            Positioned(bottom: _bottom, left: _left, child: Icon(Icons.circle)),
          ],
        ),
        decoration: BoxDecoration(
          image: DecorationImage(
            image: _garageImage,
            fit: BoxFit.cover,
          ),
        ));
  }
}
